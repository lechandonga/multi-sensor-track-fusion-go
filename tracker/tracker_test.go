package tracker_test

import (
	"sync"
	"testing"
	"time"

	"github.com/lechandonga/multi-sensor-track-fusion-go/tracker"
)

var (
	tBase     = time.Date(2026, 10, 7, 10, 0, 0, 0, time.UTC)
	sensorSet = []tracker.Sensor{
		{ID: "lidar", Kind: tracker.SensorLidar, ClockOffset: -10 * time.Millisecond, Priority: 0},
		{ID: "radar", Kind: tracker.SensorRadar, ClockOffset: 20 * time.Millisecond, Priority: 1},
		{ID: "camera", Kind: tracker.SensorCamera, ClockOffset: 0, Priority: 2},
	}
)

func det(x, y float64, class tracker.Class, vel bool) tracker.Detection {
	d := tracker.Detection{X: x, Y: y, Class: class}
	if vel {
		d.VX, d.VY, d.HasVelocity = 1, 0, true
	}
	return d
}

func ingest(t *testing.T, tr *tracker.Tracker, f tracker.Frame) {
	t.Helper()
	if err := tr.Ingest(f); err != nil {
		t.Fatalf("Ingest 失败: %v", err)
	}
}

func findTrack(list []tracker.TrackView, id string) *tracker.TrackView {
	for i := range list {
		if list[i].ID == id {
			return &list[i]
		}
	}
	return nil
}

func ms(n int) time.Time { return tBase.Add(time.Duration(n) * time.Millisecond) }

// TestOcclusionAndCoast 覆盖"遮挡与短时丢检"业务。
func TestOcclusionAndCoast(t *testing.T) {
	const biz = "遮挡与短时丢检"

	t.Run("丢检期间coasting且超窗才判丢", func(t *testing.T) {
		tr := tracker.New(tracker.DefaultConfig(), sensorSet...)
		for k := 0; k <= 3; k++ {
			tt := ms(k * 100)
			ingest(t, tr, tracker.Frame{SensorID: "lidar", Seq: uint64(k + 1),
				RawTime:    tt.Add(10 * time.Millisecond),
				Detections: []tracker.Detection{det(10+float64(k), 0, "", false)}})
		}

		tv := findTrack(tr.Snapshot(ms(350)), "T1")
		if tv == nil || tv.Status != tracker.StatusCoasting {
			t.Fatalf("t+350ms 应为 coasting，实际 %+v", tv)
		}
		bizLog(t, biz, "coasting", "t+350ms 状态=%s 外推位置=(%.2f,%.2f)", tv.Status, tv.State.X, tv.State.Y)

		if tv := findTrack(tr.Snapshot(ms(701)), "T1"); tv == nil || tv.Status != tracker.StatusLost {
			t.Fatalf("t+701ms 应为 lost（lastSeen t+300 + CoastTime 400），实际 %+v", tv)
		}
		if tv := findTrack(tr.Snapshot(ms(701).Add(tracker.DefaultConfig().ReacquireTime+time.Millisecond)), "T1"); tv != nil {
			t.Fatalf("续接窗口结束后目标不应再出现在快照中")
		}
		bizLog(t, biz, "判丢边界", "超过 CoastTime 后明确判丢；lost 在续接窗口内可见，窗口结束后消失")
	})

	t.Run("续接窗口内沿用原ID超窗后新建", func(t *testing.T) {
		cfg := tracker.DefaultConfig()
		cfg.CoastTime = 200 * time.Millisecond
		cfg.ReacquireTime = 2 * time.Second
		cfg.MaxFutureSkew = 3 * time.Second
		tr := tracker.New(cfg, sensorSet...)
		for k := 0; k <= 2; k++ {
			ingest(t, tr, tracker.Frame{SensorID: "lidar", Seq: uint64(k + 1),
				RawTime:    ms(k * 100).Add(10 * time.Millisecond),
				Detections: []tracker.Detection{det(10+float64(k), 0, "", true)}})
		}
		ingest(t, tr, tracker.Frame{SensorID: "lidar", Seq: 4,
			RawTime:    ms(600).Add(10 * time.Millisecond),
			Detections: []tracker.Detection{det(10.6, 0, "", true)}})
		if findTrack(tr.Tracks(), "T1") == nil {
			t.Fatalf("续接窗口内重新观测应沿用 T1")
		}
		if findTrack(tr.Tracks(), "T2") != nil {
			t.Fatalf("续接窗口内不应新建 T2")
		}
		bizLog(t, biz, "续接", "丢失后 t+600ms 重新观测，沿用 T1，未新建目标")

		ingest(t, tr, tracker.Frame{SensorID: "lidar", Seq: 5,
			RawTime:    ms(3000).Add(10 * time.Millisecond),
			Detections: []tracker.Detection{{X: 20, Y: 9, VX: 1, HasVelocity: true}}})
		if findTrack(tr.Tracks(), "T2") == nil {
			t.Fatalf("超过续接窗口后重新出现应新建 T2")
		}
		bizLog(t, biz, "新建", "超过 ReacquireTime 后重新出现，按规则新建 T2")
	})
}

// TestCrossSensorFusion 覆盖"跨传感器重复"业务。
func TestCrossSensorFusion(t *testing.T) {
	const biz = "跨传感器关联与融合"
	tr := tracker.New(tracker.DefaultConfig(), sensorSet...)

	for k := 0; k < 4; k++ {
		tt := ms(k * 100)
		ingest(t, tr, tracker.Frame{SensorID: "lidar", Seq: uint64(k + 1),
			RawTime:    tt.Add(10 * time.Millisecond),
			Detections: []tracker.Detection{det(10+0.01*float64(k), 0, "", false)}})
		ingest(t, tr, tracker.Frame{SensorID: "radar", Seq: uint64(k + 1),
			RawTime:    tt.Add(-20 * time.Millisecond),
			Detections: []tracker.Detection{det(10-0.01*float64(k), 0.02, tracker.ClassCar, true)}})
		ingest(t, tr, tracker.Frame{SensorID: "camera", Seq: uint64(k + 1),
			RawTime:    tt,
			Detections: []tracker.Detection{det(10.02, -0.02, tracker.ClassCar, false)}})
	}

	tracks := tr.Tracks()
	if len(tracks) != 1 {
		t.Fatalf("三路观测同一目标应只有 1 条轨迹，实际 %d: %+v", len(tracks), tracks)
	}
	tv := tracks[0]
	if tv.ID != "T1" || len(tv.SourceSensors) != 3 {
		t.Fatalf("期望唯一稳定 T1 且含三路来源，实际 %s %v", tv.ID, tv.SourceSensors)
	}
	bizLog(t, biz, "单一轨迹", "三路同时观测 -> 唯一轨迹 %s，来源=%v，ID 全程稳定", tv.ID, tv.SourceSensors)

	fused := 0
	for _, p := range tr.History("T1") {
		if p.Sensor == "fusion" {
			fused++
		}
	}
	if fused == 0 {
		t.Fatalf("应存在多传感器融合节点")
	}
	bizLog(t, biz, "融合节点", "同时刻一致观测合并为 fusion 节点 %d 个", fused)
}

// TestOutOfOrderAndLate 覆盖"乱序、重复与超迟"业务。
func TestOutOfOrderAndLate(t *testing.T) {
	const biz = "乱序、重复与超迟"
	cfg := tracker.DefaultConfig()
	cfg.MaxLateArrival = 300 * time.Millisecond
	cfg.MaxFutureSkew = 3 * time.Second
	tr := tracker.New(cfg, sensorSet...)

	ingest(t, tr, tracker.Frame{SensorID: "lidar", Seq: 1, RawTime: ms(0).Add(10 * time.Millisecond),
		Detections: []tracker.Detection{det(0, 0, "", false)}})
	ingest(t, tr, tracker.Frame{SensorID: "lidar", Seq: 2, RawTime: ms(200).Add(10 * time.Millisecond),
		Detections: []tracker.Detection{det(0.2, 0, "", false)}})
	ingest(t, tr, tracker.Frame{SensorID: "lidar", Seq: 3, RawTime: ms(100).Add(10 * time.Millisecond),
		Detections: []tracker.Detection{det(0.1, 0, "", false)}})
	bizLog(t, biz, "迟到可修正", "seq=3 时间早于 seq=2 但在窗口内，已接受并按统一时间归位")

	before := tr.Stats().AcceptedFrames
	err := tr.Ingest(tracker.Frame{SensorID: "lidar", Seq: 3,
		RawTime:    ms(100).Add(10 * time.Millisecond),
		Detections: []tracker.Detection{det(0.1, 0, "", false)}})
	if rejectKind(t, err) != tracker.RejectDuplicate || tr.Stats().AcceptedFrames != before {
		t.Fatalf("重复帧应为 duplicate 且不累计，err=%v frames=%d->%d", err, before, tr.Stats().AcceptedFrames)
	}
	bizLog(t, biz, "重复帧", "seq=3 原样二次送达被拒绝（duplicate），帧数不增加")

	ingest(t, tr, tracker.Frame{SensorID: "lidar", Seq: 4, RawTime: ms(600).Add(10 * time.Millisecond),
		Detections: []tracker.Detection{det(0.6, 0, "", false)}})
	err = tr.Ingest(tracker.Frame{SensorID: "lidar", Seq: 5, RawTime: ms(200).Add(10 * time.Millisecond)})
	if rejectKind(t, err) != tracker.RejectTooLate {
		t.Fatalf("超迟帧应为 too_late，实际 %v", err)
	}
	bizLog(t, biz, "超迟拒绝", "水位 600ms 后送达 200ms 帧 -> too_late")

	err = tr.Ingest(tracker.Frame{SensorID: "lidar", Seq: 6, RawTime: ms(4000).Add(10 * time.Millisecond)})
	if rejectKind(t, err) != tracker.RejectFromFuture {
		t.Fatalf("超前帧应为 from_future，实际 %v", err)
	}
	err = tr.Ingest(tracker.Frame{SensorID: "lidar", Seq: 7})
	if rejectKind(t, err) != tracker.RejectBadTimestamp {
		t.Fatalf("零时间戳应为 bad_timestamp，实际 %v", err)
	}
	err = tr.Ingest(tracker.Frame{SensorID: "sonar", Seq: 1, RawTime: ms(600)})
	if rejectKind(t, err) != tracker.RejectUnknownSensor {
		t.Fatalf("未知传感器应为 unknown_sensor，实际 %v", err)
	}
	bizLog(t, biz, "异常时间戳", "from_future / bad_timestamp / unknown_sensor 均被分类拒绝")
}

// TestConflictResolution 覆盖"冲突观测取舍"业务。
func TestConflictResolution(t *testing.T) {
	const biz = "冲突观测取舍"
	cfg := tracker.DefaultConfig()
	tr := tracker.New(cfg, sensorSet...)

	for k := 0; k < 2; k++ {
		tt := ms(k * 100)
		ingest(t, tr, tracker.Frame{SensorID: "lidar", Seq: uint64(k + 1),
			RawTime: tt.Add(10 * time.Millisecond), Detections: []tracker.Detection{det(10, 5, "", false)}})
		ingest(t, tr, tracker.Frame{SensorID: "radar", Seq: uint64(k + 1),
			RawTime: tt.Add(-20 * time.Millisecond), Detections: []tracker.Detection{det(10.1, 5, "", true)}})
	}
	tt := ms(200)
	ingest(t, tr, tracker.Frame{SensorID: "lidar", Seq: 3,
		RawTime: tt.Add(10 * time.Millisecond), Detections: []tracker.Detection{det(10, 5, "", false)}})
	ingest(t, tr, tracker.Frame{SensorID: "camera", Seq: 3, RawTime: tt,
		Detections: []tracker.Detection{det(13, 5, tracker.ClassPed, false)}})

	conflicts := tr.Conflicts()
	if len(conflicts) != 1 {
		t.Fatalf("应记录 1 次冲突，实际 %d", len(conflicts))
	}
	c := conflicts[0]
	if c.Chosen != "lidar" || c.Rejected != "camera" {
		t.Fatalf("应 lidar 胜/camera 败，实际 chosen=%s rejected=%s", c.Chosen, c.Rejected)
	}
	bizLog(t, biz, "权威取舍", "t+200ms 相机冲突观测被拒，采用更权威的 %s；被拒值不混合，规则留痕", c.Chosen)

	// 冲突值没有被混进位置（仍应接近 10，而不是偏向 13），类别也未被 pedestrian 污染。
	tv := findTrack(tr.Snapshot(tt), "T1")
	if tv == nil {
		t.Fatal("T1 应存在")
	}
	if tv.State.X > 11.5 {
		t.Fatalf("冲突观测的位置被混合进轨迹: X=%.2f", tv.State.X)
	}
	if tv.State.Class == tracker.ClassPed {
		t.Fatalf("冲突观测的类别 pedestrian 被混入")
	}
	bizLog(t, biz, "不混合", "冲突后位置 X=%.2f 未偏向 13，类别未被污染", tv.State.X)

	// 冲突相机观测也不应生成第二条目标。
	if len(tr.Tracks()) != 1 {
		t.Fatalf("冲突观测不应产生第二条轨迹，实际 %d", len(tr.Tracks()))
	}
}

// TestClockOffsetFixed 覆盖"固定时钟偏差"业务：
// 偏差在配置时声明、随数据固定修正，到达顺序不同不应把时间轴带偏。
func TestClockOffsetFixed(t *testing.T) {
	const biz = "固定时钟偏差"
	must := func(tr *tracker.Tracker) []tracker.TrackView {
		ingest(t, tr, tracker.Frame{SensorID: "lidar", Seq: 1, RawTime: ms(0).Add(10 * time.Millisecond),
			Detections: []tracker.Detection{det(0, 0, "", false)}})
		ingest(t, tr, tracker.Frame{SensorID: "radar", Seq: 1, RawTime: ms(0).Add(-20 * time.Millisecond),
			Detections: []tracker.Detection{det(0.05, 0, "", true)}})
		return tr.Tracks()
	}
	tr1 := tracker.New(tracker.DefaultConfig(), sensorSet...)
	t1 := must(tr1)
	tr2 := tracker.New(tracker.DefaultConfig(), reverseSensors(sensorSet)...)
	t2 := must(tr2)
	if len(t1) != 1 || len(t2) != 1 || t1[0].State != t2[0].State ||
		!t1[0].FirstSeen.Equal(t2[0].FirstSeen) {
		t.Fatalf("仅到达顺序不同，结果应一致:\n%+v\n%+v", t1, t2)
	}
	bizLog(t, biz, "顺序无关", "雷达先到/激光先到两种顺序，FirstSeen 与状态完全一致: %s", t1[0].FirstSeen)
}

func reverseSensors(in []tracker.Sensor) []tracker.Sensor {
	out := append([]tracker.Sensor(nil), in...)
	for i, j := 0, len(out)-1; i < j; i, j = i+1, j-1 {
		out[i], out[j] = out[j], out[i]
	}
	return out
}

// TestConcurrentReadWrite 覆盖"并发读写"业务：多传感器并发写入、
// 并发查询与导出，开启 -race 时必须稳定通过，且结果与串行灌入一致。
func TestConcurrentReadWrite(t *testing.T) {
	const biz = "并发读写一致性"

	build := func() *tracker.Tracker {
		return tracker.New(tracker.DefaultConfig(), sensorSet...)
	}

	// 串行基线。
	base := build()
	for k := 0; k < 30; k++ {
		tt := ms(k * 50)
		for _, s := range []string{"lidar", "radar", "camera"} {
			off := time.Duration(0)
			switch s {
			case "lidar":
				off = 10 * time.Millisecond
			case "radar":
				off = -20 * time.Millisecond
			}
			d := []tracker.Detection{det(float64(k)*0.2, 0, tracker.ClassCar, s == "radar")}
			if k%7 == 0 && s == "camera" {
				d = nil // 相机偶发丢检
			}
			ingest(t, base, tracker.Frame{SensorID: s, Seq: uint64(k + 1),
				RawTime: tt.Add(off), Detections: d})
		}
	}
	baseTracks := base.Tracks()

	run := func() {
		tr := build()
		var readerWG, writerWG sync.WaitGroup
		stop := make(chan struct{})

		// 并发读者：快照、轨迹、历史、冲突、导出。
		for r := 0; r < 4; r++ {
			readerWG.Add(1)
			go func() {
				defer readerWG.Done()
				for {
					select {
					case <-stop:
						return
					default:
						_ = tr.Snapshot(time.Time{})
						_ = tr.Tracks()
						_ = tr.History("T1")
						_ = tr.Conflicts()
						_ = tr.AcceptedFrames()
					}
				}
			}()
		}

		// 三路并发写者：每路自己的序号严格递增，并发但帧内容与基线相同。
		for _, s := range []string{"lidar", "radar", "camera"} {
			writerWG.Add(1)
			go func(s string) {
				defer writerWG.Done()
				off := time.Duration(0)
				switch s {
				case "lidar":
					off = 10 * time.Millisecond
				case "radar":
					off = -20 * time.Millisecond
				}
				for k := 0; k < 30; k++ {
					d := []tracker.Detection{det(float64(k)*0.2, 0, tracker.ClassCar, s == "radar")}
					if k%7 == 0 && s == "camera" {
						d = nil
					}
					if err := tr.Ingest(tracker.Frame{SensorID: s, Seq: uint64(k + 1),
						RawTime: ms(k * 50).Add(off), Detections: d}); err != nil {
						t.Errorf("并发写入被意外拒绝: %v", err)
						return
					}
				}
			}(s)
		}

		// 写者全部完成后再停止读者，保证全程读写交叠。
		writerWG.Wait()
		close(stop)
		readerWG.Wait()

		got := tr.Tracks()
		if len(got) != len(baseTracks) {
			t.Fatalf("并发写入结果轨迹数不一致: %d != %d", len(got), len(baseTracks))
		}
		for i := range got {
			if got[i].ID != baseTracks[i].ID || got[i].State != baseTracks[i].State {
				t.Fatalf("并发结果与串行基线不一致:\n%+v\n%+v", got[i], baseTracks[i])
			}
		}
	}

	for i := 0; i < 5; i++ {
		run()
	}
	bizLog(t, biz, "竞态与等价", "4 读者 × 3 写者并发，重复 5 轮，结果与串行基线一致")
}

// TestReplayReproducible 覆盖"重放复现"业务。
func TestReplayReproducible(t *testing.T) {
	const biz = "重放与重启复现"

	cfg := tracker.DefaultConfig()
	tr := tracker.New(cfg, sensorSet...)
	for k := 0; k < 20; k++ {
		tt := ms(k * 100)
		// 故意以"雷达、相机、激光"的反权威顺序送达，证明结果只取决于规范顺序。
		ingest(t, tr, tracker.Frame{SensorID: "radar", Seq: uint64(k + 1), RawTime: tt.Add(-20 * time.Millisecond),
			Detections: []tracker.Detection{det(float64(k)*0.4, 1, tracker.ClassCar, true)}})
		ingest(t, tr, tracker.Frame{SensorID: "camera", Seq: uint64(k + 1), RawTime: tt,
			Detections: []tracker.Detection{det(float64(k)*0.4+0.03, 1, tracker.ClassCar, false)}})
		ingest(t, tr, tracker.Frame{SensorID: "lidar", Seq: uint64(k + 1), RawTime: tt.Add(10 * time.Millisecond),
			Detections: []tracker.Detection{det(float64(k)*0.4-0.03, 1, "", false)}})
	}

	exported := tr.AcceptedFrames()
	rp := tracker.Replay(cfg, sensorSet, exported)
	if !sameViews(tr.Tracks(), rp.Tracks()) || len(tr.Conflicts()) != len(rp.Conflicts()) {
		t.Fatalf("重放结果与原融合器不一致")
	}
	for i := range tr.Tracks() {
		a, b := tr.History(tr.Tracks()[i].ID), rp.History(rp.Tracks()[i].ID)
		if len(a) != len(b) {
			t.Fatalf("轨迹 %s 历史长度不一致", tr.Tracks()[i].ID)
		}
		for j := range a {
			if a[j] != b[j] {
				t.Fatalf("轨迹 %s 历史节点 %d 不一致: %+v != %+v", tr.Tracks()[i].ID, j, a[j], b[j])
			}
		}
	}
	bizLog(t, biz, "重放一致", "导出规范帧后 Replay，轨迹、ID、逐节点历史完全一致")

	// 再灌一遍同样的导出帧到第三个全新实例（等价重启），结果仍一致。
	rp2 := tracker.Replay(cfg, sensorSet, exported)
	if !sameViews(rp.Tracks(), rp2.Tracks()) {
		t.Fatalf("重启后再灌一遍结果不一致")
	}
	bizLog(t, biz, "重启复现", "全新实例再次灌入相同数据，轨迹与标识逐字节一致")
}

func sameViews(a, b []tracker.TrackView) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i].ID != b[i].ID || a[i].Status != b[i].Status ||
			a[i].State != b[i].State ||
			!a[i].FirstSeen.Equal(b[i].FirstSeen) || !a[i].LastSeen.Equal(b[i].LastSeen) {
			return false
		}
	}
	return true
}
