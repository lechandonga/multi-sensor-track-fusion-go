package tracker

import (
	"fmt"
	"math"
	"sort"
	"time"
)

// canonicalFrameLess 定义帧的唯一规范处理顺序：
// 统一时间升序；同时间按传感器权威优先级升序；再按传感器 ID、序号升序。
// 该顺序与帧的到达顺序无关，是重放可复现的基础。
func canonicalFrameLess(a, b acceptedFrame) bool {
	if !a.t.Equal(b.t) {
		return a.t.Before(b.t)
	}
	if a.sensor.Priority != b.sensor.Priority {
		return a.sensor.Priority < b.sensor.Priority
	}
	if a.sensor.ID != b.sensor.ID {
		return a.sensor.ID < b.sensor.ID
	}
	return a.seq < b.seq
}

// rebuildLocked 以当前全部已接受帧为输入，从零、纯函数式重建轨迹。
func (t *Tracker) rebuildLocked() {
	frames := make([]acceptedFrame, len(t.frames))
	copy(frames, t.frames)
	sort.SliceStable(frames, func(i, j int) bool {
		return canonicalFrameLess(frames[i], frames[j])
	})

	t.tracks = nil
	t.nextIDn = 0
	t.conflicts = nil

	for fi := range frames {
		f := frames[fi]
		t.processFrameLocked(f)
	}

	// 收尾：候选超时丢弃；确认轨迹超过内存保留窗口后剔除
	//（续接窗口 ReacquireTime 之后不再可续接，DeadHistoryTime 之后回收）。
	kept := t.tracks[:0]
	for _, tr := range t.tracks {
		if !tr.confirmed {
			if t.maxTime.Sub(tr.lastSeen) > t.cfg.TentativeTTL {
				continue
			}
			kept = append(kept, tr)
			continue
		}
		if t.maxTime.Sub(tr.lostAt) > t.cfg.DeadHistoryTime {
			continue
		}
		kept = append(kept, tr)
	}
	t.tracks = kept
}

type cand struct {
	tr   *track
	det  int
	d    float64
	lost bool // 关联的是处于丢失/续接窗口的轨迹
}

func (t *Tracker) processFrameLocked(f acceptedFrame) {
	used := make(map[*track]bool)
	detUsed := make(map[int]bool)

	var cands []cand
	for _, tr := range t.tracks {
		live := t.liveAtLocked(tr, f.t)
		if !live.associable {
			continue
		}
		pred := predictState(tr.state, f.t.Sub(tr.lastUpdate))
		for di, det := range f.detections {
			dx := det.X - pred.X
			dy := det.Y - pred.Y
			d := math.Sqrt(dx*dx+dy*dy) / live.sigma
			if d <= t.cfg.GateRadius {
				cands = append(cands, cand{tr: tr, det: di, d: d, lost: live.lost})
			}
		}
	}

	// 贪心全局最近邻：距离升序，距离相同按轨迹 ID、传感器权威、检测序号决胜。
	sort.SliceStable(cands, func(i, j int) bool {
		a, b := cands[i], cands[j]
		if a.d != b.d {
			return a.d < b.d
		}
		if a.tr.id != b.tr.id {
			return a.tr.id < b.tr.id
		}
		return a.det < b.det
	})

	for _, c := range cands {
		if used[c.tr] || detUsed[c.det] {
			continue
		}
		det := f.detections[c.det]

		// 冲突判定：同一时刻已有另一路传感器的观测，且两条观测空间差过大。
		if winner, sameTS, obs := t.assignedAtLocked(c.tr, f.t, f.sensor); sameTS {
			od := obsDistance(det, obs) / t.cfg.PositionSigma
			if od > t.cfg.ConflictRadius {
				// 规范顺序保证已采用者权威不低于当前帧，故当前观测被拒绝。
				t.conflicts = append(t.conflicts, Conflict{
					Time:      f.t,
					TrackID:   c.tr.id,
					Chosen:    winner,
					Rejected:  f.sensor.ID,
					Metric:    "position",
					ChosenVal: obs.X,
					OtherVal:  det.X,
					Rule: "same-time observations differ beyond conflict radius; " +
						"higher-priority sensor wins, rejected value is never blended",
				})
				detUsed[c.det] = true
				continue
			}
		}

		t.fuseLocked(c.tr, f, det, c.lost)
		used[c.tr] = true
		detUsed[c.det] = true
	}

	// 未分配检测：先尝试续接丢失轨迹，否则新建候选目标。
	for di, det := range f.detections {
		if detUsed[di] {
			continue
		}
		if tr := t.tryReacquireLocked(f, det); tr != nil {
			t.fuseLocked(tr, f, det, true)
			continue
		}
		t.newTrackLocked(f, det)
	}
}

type liveness struct {
	associable bool
	lost       bool
	sigma      float64
}

func (t *Tracker) liveAtLocked(tr *track, at time.Time) liveness {
	dt := at.Sub(tr.lastUpdate).Seconds()
	sigma := math.Sqrt(t.cfg.PositionSigma*t.cfg.PositionSigma +
		math.Pow(t.cfg.ProcessSigma*math.Max(dt, 0), 2))
	if !tr.confirmed {
		if at.Sub(tr.lastSeen) > t.cfg.TentativeTTL {
			return liveness{}
		}
		return liveness{associable: true, sigma: sigma}
	}
	if !at.After(tr.deadAt) {
		return liveness{associable: true, lost: at.After(tr.lostAt), sigma: sigma}
	}
	return liveness{}
}

// assignedAtLocked 查询 tr 在时刻 ts 是否已经采用了另一路传感器的观测。
func (t *Tracker) assignedAtLocked(tr *track, ts time.Time, self Sensor) (string, bool, Detection) {
	if tr.lastUpdate.Equal(ts) && tr.lastWinner != "" && tr.lastWinner != self.ID {
		return tr.lastWinner, true, tr.lastWinnerObs
	}
	return "", false, Detection{}
}

func (t *Tracker) fuseLocked(tr *track, f acceptedFrame, det Detection, reacquired bool) {
	note := ""
	if reacquired {
		note = "reacquired after coasting"
	}

	first := tr.hits == 0
	if !first {
		pred := predictState(tr.state, f.t.Sub(tr.lastUpdate))
		a := t.cfg.FusionAlpha
		tr.state.X = pred.X + a*(det.X-pred.X)
		tr.state.Y = pred.Y + a*(det.Y-pred.Y)
		switch {
		case det.HasVelocity:
			tr.state.VX = pred.VX + a*(det.VX-pred.VX)
			tr.state.VY = pred.VY + a*(det.VY-pred.VY)
		case f.t.After(tr.lastSeen):
			gap := f.t.Sub(tr.lastSeen).Seconds()
			estVX := (det.X - tr.lastRawX) / gap
			estVY := (det.Y - tr.lastRawY) / gap
			tr.state.VX = pred.VX + a*0.5*(estVX-pred.VX)
			tr.state.VY = pred.VY + a*0.5*(estVY-pred.VY)
		}
	} else {
		tr.state.X = det.X
		tr.state.Y = det.Y
		tr.state.VX = det.VX
		tr.state.VY = det.VY
		tr.firstSeen = f.t
	}

	// 类别固定权重投票：权重 = 1/(优先级+1)，只在命中时累计。
	if det.Class != ClassUnknown {
		if tr.classVotes == nil {
			tr.classVotes = map[Class]float64{}
		}
		tr.classVotes[det.Class] += 1.0 / float64(f.sensor.Priority+1)
		tr.state.Class = leadingClass(tr.classVotes)
	} else if first {
		tr.state.Class = ClassUnknown
	}

	tr.hits++
	if !tr.confirmed && tr.hits >= t.cfg.ConfirmHits {
		tr.confirmed = true
	}

	if !containsStr(tr.sources, f.sensor.ID) {
		tr.sources = append(tr.sources, f.sensor.ID)
	}

	tr.lastSeen = f.t
	tr.lastUpdate = f.t
	tr.lastRawX = det.X
	tr.lastRawY = det.Y
	tr.lastWinner = f.sensor.ID
	tr.lastWinnerObs = det
	if tr.confirmed {
		tr.lostAt = f.t.Add(t.cfg.CoastTime)
		tr.deadAt = tr.lostAt.Add(t.cfg.ReacquireTime)
	}

	label := f.sensor.ID
	if n := len(tr.history); n > 0 && tr.history[n-1].Time.Equal(f.t) {
		// 同一时刻第二路（或更多）传感器的一致观测：记为融合节点。
		label = "fusion"
		hp := tr.history[n-1]
		hp.Sensor = label
		if note != "" {
			hp.Note = note
		}
		hp.State = tr.state
		hp.Hits = tr.hits
		tr.history[n-1] = hp
		return
	}
	tr.history = append(tr.history, HistoryPoint{
		Time: f.t, State: tr.state, Sensor: label, Note: note, Hits: tr.hits,
	})
}

func (t *Tracker) tryReacquireLocked(f acceptedFrame, det Detection) *track {
	var best *track
	bestD := math.Inf(1)
	for _, tr := range t.tracks {
		if !tr.confirmed || !f.t.After(tr.lostAt) || f.t.After(tr.deadAt) {
			continue
		}
		pred := predictState(tr.state, f.t.Sub(tr.lastUpdate))
		sigma := math.Sqrt(t.cfg.PositionSigma*t.cfg.PositionSigma +
			math.Pow(t.cfg.ProcessSigma*f.t.Sub(tr.lastUpdate).Seconds(), 2))
		d := math.Sqrt((det.X-pred.X)*(det.X-pred.X)+
			(det.Y-pred.Y)*(det.Y-pred.Y)) / sigma
		if d <= t.cfg.GateRadius && (d < bestD || (d == bestD && best != nil && tr.id < best.id)) {
			best, bestD = tr, d
		}
	}
	return best
}

func (t *Tracker) newTrackLocked(f acceptedFrame, det Detection) {
	t.nextIDn++
	tr := &track{id: fmt.Sprintf("%s%d", t.cfg.TrackIDPrefix, t.nextIDn)}
	tr.lastUpdate = f.t
	t.tracks = append(t.tracks, tr)
	t.fuseLocked(tr, f, det, false)
}

// leadingClass 返回累计权重最高的类别；平票按类别名字典序决胜，保证确定性。
func leadingClass(votes map[Class]float64) Class {
	var best Class
	bestW := math.Inf(-1)
	classes := make([]Class, 0, len(votes))
	for c := range votes {
		classes = append(classes, c)
	}
	sort.Strings(toStrings(classes))
	for _, c := range classes {
		if votes[c] > bestW {
			best, bestW = c, votes[c]
		}
	}
	return best
}

func predictState(s State, dt time.Duration) State {
	d := dt.Seconds()
	return State{
		X:     s.X + s.VX*d,
		Y:     s.Y + s.VY*d,
		VX:    s.VX,
		VY:    s.VY,
		Class: s.Class,
	}
}

func obsDistance(a, b Detection) float64 {
	return math.Sqrt((a.X-b.X)*(a.X-b.X) + (a.Y-b.Y)*(a.Y-b.Y))
}

func containsStr(xs []string, x string) bool {
	for _, v := range xs {
		if v == x {
			return true
		}
	}
	return false
}

func toStrings(cs []Class) []string {
	out := make([]string, len(cs))
	for i, c := range cs {
		out[i] = string(c)
	}
	return out
}
