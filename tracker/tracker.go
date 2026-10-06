// Package tracker 实现一套纯本地、确定性、可复现的多传感器目标跟踪融合组件。
//
// 输入帧先做时钟修正（固定偏差，不做在线估计），再按统一时间的规范顺序
// 以纯函数方式重建轨迹；同一段输入（即使乱序到达）重放两次结果一致。
// 所有读写由同一把读写锁保护，查询永远只看到完整一致的快照。
package tracker

import (
	"sort"
	"sync"
	"time"
)

// acceptedFrame 是通过校验、时钟已修正的帧。
type acceptedFrame struct {
	sensor     Sensor
	seq        uint64
	t          time.Time
	detections []Detection
}

// track 是内部目标状态。
type track struct {
	id         string
	state      State
	firstSeen  time.Time
	lastSeen   time.Time
	lastUpdate time.Time
	hits       int
	confirmed  bool
	lostAt     time.Time // == lastSeen + CoastTime，仅对 confirmed 有意义
	deadAt     time.Time // lostAt + ReacquireTime，之后不可续接
	sources    []string
	history    []HistoryPoint
	// classVotes 类别投票：class -> 累计传感器权重。
	classVotes map[Class]float64
	// lastWinner 最近一次冲突中被采用的传感器 ID（空表示无冲突）。
	lastWinner    string
	lastWinnerObs Detection
	// lastRawX/Y 上一次观测的原始位置，用于无速度传感器的速度估计。
	lastRawX float64
	lastRawY float64
}

// HistoryPoint 是轨迹上的一个确定节点，用于解释与审计。
type HistoryPoint struct {
	Time   time.Time
	State  State
	Sensor string // "fusion" 表示多传感器融合；否则为采用的传感器 ID
	Note   string
	Hits   int // 截至该节点的累计命中次数（用于 as-of 判定是否已确认）
}

// Conflict 记录一次被明确取舍的传感器冲突，供解释使用。
type Conflict struct {
	Time      time.Time
	TrackID   string
	Chosen    string
	Rejected  string
	Metric    string
	ChosenVal float64
	OtherVal  float64
	Rule      string
}

// Tracker 是融合器。零值不可用，请用 New。
type Tracker struct {
	mu      sync.RWMutex
	cfg     Config
	sensors map[string]Sensor

	frames   []acceptedFrame // 规范顺序见 canonicalFrameLess
	lastSeq  map[string]uint64
	lastTime map[string]time.Time

	tracks  []*track // 当前重建结果（dead 已剔除）
	nextIDn int
	maxTime time.Time

	conflicts []Conflict
}

// New 创建融合器并注册传感器。
func New(cfg Config, sensors ...Sensor) *Tracker {
	t := &Tracker{
		cfg:      cfg,
		sensors:  make(map[string]Sensor),
		lastSeq:  make(map[string]uint64),
		lastTime: make(map[string]time.Time),
	}
	for _, s := range sensors {
		t.sensors[s.ID] = s
	}
	return t
}

// SensorIDs 返回已注册传感器 ID（按优先级、再按 ID 排序，保证确定性）。
func (t *Tracker) SensorIDs() []string {
	t.mu.RLock()
	defer t.mu.RUnlock()
	return t.sensorIDsLocked()
}

func (t *Tracker) sensorIDsLocked() []string {
	ids := make([]string, 0, len(t.sensors))
	for id := range t.sensors {
		ids = append(ids, id)
	}
	sort.Slice(ids, func(i, j int) bool {
		a, b := t.sensors[ids[i]], t.sensors[ids[j]]
		if a.Priority != b.Priority {
			return a.Priority < b.Priority
		}
		return ids[i] < ids[j]
	})
	return ids
}

// Ingest 接收一帧。乱序到达允许（只要仍在可修正窗口内）；重复帧幂等忽略
// 并返回 *FrameRejectedError{Kind: RejectDuplicate}；各类非法帧均被明确拒绝。
func (t *Tracker) Ingest(f Frame) error {
	t.mu.Lock()
	defer t.mu.Unlock()

	s, ok := t.sensors[f.SensorID]
	if !ok {
		return &FrameRejectedError{RejectUnknownSensor, f.SensorID, f.Seq,
			"sensor is not registered"}
	}
	if f.RawTime.IsZero() {
		return &FrameRejectedError{RejectBadTimestamp, f.SensorID, f.Seq,
			"zero timestamp"}
	}
	ct := f.correctedTime(s.ClockOffset)

	// 重复帧定义：同一传感器、同一序号即视为重复投递，幂等拒绝，
	// 不因其携带的数据或送达次数而重复累计。
	if prevSeq, seen := t.lastSeq[f.SensorID]; seen && f.Seq <= prevSeq {
		return &FrameRejectedError{RejectDuplicate, f.SensorID, f.Seq,
			"frame sequence number already accepted (duplicate delivery)"}
	}

	// 未来时间一律以全局水位衡量。
	if !t.maxTime.IsZero() && ct.After(t.maxTime.Add(t.cfg.MaxFutureSkew)) {
		return &FrameRejectedError{RejectFromFuture, f.SensorID, f.Seq,
			"corrected timestamp is too far in the future"}
	}

	// 迟到窗口按传感器各自的时间轴衡量：多路并发、各有自己的节奏，
	// 不能因为另一路时钟领先就把这路的正常首帧/帧误判为超迟。
	// 该路已有帧时：倒退超过窗口 -> too_late；原地不动（且同序号） ->
	// duplicate；窗口内倒退 -> out_of_order，但仍在可修正范围，接受。
	if prev, seen := t.lastTime[f.SensorID]; seen {
		switch {
		case ct.Before(prev.Add(-t.cfg.MaxLateArrival)):
			return &FrameRejectedError{RejectTooLate, f.SensorID, f.Seq,
				"corrected timestamp " + ct.Format(time.RFC3339Nano) +
					" is earlier than this sensor's watermark minus max-late-arrival"}
		case !ct.After(prev):
			// 窗口内乱序（序号已更新、时间倒退）：属于可修正迟到，予以接受，
			// 由规范排序保证时间轴不被送达顺序带偏。
		}
	}

	af := acceptedFrame{
		sensor:     s,
		seq:        f.Seq,
		t:          ct,
		detections: append([]Detection(nil), f.Detections...),
	}
	t.frames = append(t.frames, af)
	t.lastSeq[f.SensorID] = f.Seq
	t.lastTime[f.SensorID] = ct
	if ct.After(t.maxTime) {
		t.maxTime = ct
	}

	t.rebuildLocked()
	return nil
}

// Snapshot 返回 queryTime 时刻一致的跟踪结果。queryTime 为零值时取当前
// 已知最大时间。候选目标在确认前不对外；已判丢目标仅在判丢时刻以 lost
// 出现一次，之后不再出现。返回值为深拷贝。
func (t *Tracker) Snapshot(queryTime time.Time) []TrackView {
	t.mu.RLock()
	defer t.mu.RUnlock()

	qt := queryTime
	if qt.IsZero() {
		qt = t.maxTime
	}
	out := make([]TrackView, 0, len(t.tracks))
	for _, tr := range t.tracks {
		st, hp, has := t.statusAt(tr, qt)
		// 查询时刻轨迹尚未产生。
		if !has {
			continue
		}
		// 候选目标在确认前不对外。
		if st == StatusTentative {
			continue
		}
		// 判丢目标在续接窗口内以 lost 可见；超过续接窗口后消失
		//（之后再出现按 ReacquireTime 规则决定续接或新建）。
		if st == StatusLost && qt.After(hp.Time.Add(t.cfg.CoastTime+t.cfg.ReacquireTime)) {
			continue
		}
		out = append(out, t.viewAt(tr, qt, st, hp))
	}
	sort.Slice(out, func(i, j int) bool { return out[i].ID < out[j].ID })
	return out
}

// Tracks 返回全部内部轨迹（含已丢失、按 ID 排序），供重放比对与审计。
func (t *Tracker) Tracks() []TrackView {
	t.mu.RLock()
	defer t.mu.RUnlock()
	out := make([]TrackView, 0, len(t.tracks))
	for _, tr := range t.tracks {
		if st, hp, has := t.statusAt(tr, t.maxTime); has {
			out = append(out, t.viewAt(tr, t.maxTime, st, hp))
		}
	}
	sort.Slice(out, func(i, j int) bool { return out[i].ID < out[j].ID })
	return out
}

// Conflicts 返回到目前为止被明确取舍的冲突记录副本。
func (t *Tracker) Conflicts() []Conflict {
	t.mu.RLock()
	defer t.mu.RUnlock()
	return append([]Conflict(nil), t.conflicts...)
}

// HistoryPointView 是对外的轨迹节点视图。
type HistoryPointView = HistoryPoint

// History 返回指定 ID 轨迹的融合节点副本（按时间升序）。
func (t *Tracker) History(trackID string) []HistoryPoint {
	t.mu.RLock()
	defer t.mu.RUnlock()
	for _, tr := range t.tracks {
		if tr.id == trackID {
			return append([]HistoryPoint(nil), tr.history...)
		}
	}
	return nil
}

// Stats 是融合器的输入统计快照。
type Stats struct {
	AcceptedFrames int
	Tracks         int
	Conflicts      int
	MaxTime        time.Time
}

// Stats 返回当前已接受帧数、轨迹数、冲突数与水位时间。
func (t *Tracker) Stats() Stats {
	t.mu.RLock()
	defer t.mu.RUnlock()
	return Stats{
		AcceptedFrames: len(t.frames),
		Tracks:         len(t.tracks),
		Conflicts:      len(t.conflicts),
		MaxTime:        t.maxTime,
	}
}

// AcceptedFrames 按规范顺序导出已接受的帧（统一时钟时间），用于离线重放。
func (t *Tracker) AcceptedFrames() []FrameExport {
	t.mu.RLock()
	defer t.mu.RUnlock()
	out := make([]FrameExport, 0, len(t.frames))
	// t.frames 按到达顺序保存；导出规范顺序，保证重放与到达顺序无关。
	idx := make([]int, len(t.frames))
	for i := range idx {
		idx[i] = i
	}
	sort.Slice(idx, func(a, b int) bool {
		return canonicalFrameLess(t.frames[idx[a]], t.frames[idx[b]])
	})
	for _, i := range idx {
		f := t.frames[i]
		out = append(out, FrameExport{
			SensorID:   f.sensor.ID,
			Seq:        f.seq,
			Time:       f.t,
			Detections: append([]Detection(nil), f.detections...),
		})
	}
	return out
}

// FrameExport 是统一时钟下的已接受帧。
type FrameExport struct {
	SensorID   string
	Seq        uint64
	Time       time.Time
	Detections []Detection
}

// Replay 用一组规范顺序的导出帧重建一个全新融合器。重放结果（轨迹、ID、
// 冲突记录）与原融合器一致；这等价于"重启后再灌一遍同样的数据"。
func Replay(cfg Config, sensors []Sensor, frames []FrameExport) *Tracker {
	// 导出帧时间已是统一时钟，重放时将所有传感器时钟偏差清零，使
	// "原始时间 + 固定偏差" 仍等于原统一时刻。
	flat := make([]Sensor, len(sensors))
	for i, s := range sensors {
		s.ClockOffset = 0
		flat[i] = s
	}
	t := New(cfg, flat...)
	for _, f := range frames {
		frame := Frame{
			SensorID:   f.SensorID,
			Seq:        f.Seq,
			RawTime:    f.Time,
			Detections: f.Detections,
		}
		if err := t.Ingest(frame); err != nil {
			// 规范导出的帧必然合法；若发生说明实现有误，直接 panic 暴露问题。
			panic(err)
		}
	}
	return t
}

// statusAt 返回 at 时刻轨迹的状态与最近的历史节点。
// 规则固定：取时间 <= at 的最后一个节点；节点命中数未达确认门槛 ->
// tentative；at 超过 lastSeen+CoastTime -> lost；介于其间 -> coasting；
// at 早于首个节点 -> 不存在。
func (t *Tracker) statusAt(tr *track, at time.Time) (TrackStatus, HistoryPoint, bool) {
	var hp HistoryPoint
	idx := -1
	for i := range tr.history {
		if !tr.history[i].Time.After(at) {
			idx = i
		} else {
			break
		}
	}
	if idx < 0 {
		return "", HistoryPoint{}, false
	}
	hp = tr.history[idx]
	if hp.Hits < t.cfg.ConfirmHits {
		return StatusTentative, hp, true
	}
	if at.After(hp.Time.Add(t.cfg.CoastTime)) {
		return StatusLost, hp, true
	}
	if at.After(hp.Time) {
		return StatusCoasting, hp, true
	}
	return StatusConfirmed, hp, true
}

func (t *Tracker) viewAt(tr *track, at time.Time, st TrackStatus, hp HistoryPoint) TrackView {
	pred := predictState(hp.State, at.Sub(hp.Time))
	missed := 0
	for _, f := range t.frames {
		if f.t.After(hp.Time) && !f.t.After(at) {
			missed++
		}
	}
	return TrackView{
		ID:            tr.id,
		Status:        st,
		State:         pred,
		FirstSeen:     tr.history[0].Time,
		LastSeen:      hp.Time,
		CoastFrames:   missed,
		SourceSensors: append([]string(nil), tr.sources...),
	}
}
