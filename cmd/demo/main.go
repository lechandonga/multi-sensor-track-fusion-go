// Command demo 在不联网、不接设备的前提下跑一遍完整模拟场景，
// 打印遮挡续接、跨传感器融合、重复/乱序/超迟拒绝、冲突取舍和重放复现结果。
package main

import (
	"errors"
	"fmt"
	"time"

	"github.com/lechandonga/multi-sensor-track-fusion-go/simulate"
	"github.com/lechandonga/multi-sensor-track-fusion-go/tracker"
)

func main() {
	t0 := time.Date(2026, 10, 7, 10, 0, 0, 0, time.UTC)
	opt := simulate.DefaultOptions()
	sc := simulate.Generate(t0, opt)

	cfg := tracker.DefaultConfig()
	tr := tracker.New(cfg, simulate.Sensors()...)

	duplicateRejected := 0
	for _, item := range sc.Delivery {
		f := sc.Frames[item.Idx]
		if err := tr.Ingest(f); err != nil {
			var rj *tracker.FrameRejectedError
			if errors.As(err, &rj) {
				if rj.Kind == tracker.RejectDuplicate {
					duplicateRejected++
					fmt.Printf("[拒绝-重复] sensor=%s seq=%d（不重复累计）\n", rj.SensorID, rj.Seq)
				} else {
					fmt.Printf("[拒绝-%s] sensor=%s seq=%d: %s\n", rj.Kind, rj.SensorID, rj.Seq, rj.Reason)
				}
			}
		}
	}

	// 再构造一帧严重超迟的帧：必须被明确拒绝并说明类别。
	stale := tracker.Frame{
		SensorID: "radar",
		Seq:      9999,
		RawTime:  t0.Add(-5 * time.Second),
	}
	err := tr.Ingest(stale)
	fmt.Printf("[拒绝-超迟演示] err=%v\n", err)

	// 再构造一帧时间戳异常（零值）的帧。
	bad := tracker.Frame{SensorID: "lidar", Seq: 9999}
	fmt.Printf("[拒绝-异常时间戳演示] err=%v\n", tr.Ingest(bad))

	stats := tr.Stats()
	fmt.Printf("\n已接受帧数=%d  轨迹数=%d  冲突记录=%d  重复帧被拒=%d\n",
		stats.AcceptedFrames, stats.Tracks, stats.Conflicts, duplicateRejected)

	fmt.Println("\n== 遮挡前后状态（目标 A）==")
	for _, probe := range []time.Duration{
		1100 * time.Millisecond, // 遮挡前
		1400 * time.Millisecond, // 遮挡中：coasting 外推
		1500 * time.Millisecond, // 恰好到达判丢时刻：lost 仅此刻可见
		1600 * time.Millisecond, // 判丢之后：快照中不再出现
		1700 * time.Millisecond, // 重新出现，续接原 ID
		1900 * time.Millisecond, // 恢复 confirmed
	} {
		at := t0.Add(probe)
		var t1 *tracker.TrackView
		snap := tr.Snapshot(at)
		for i := range snap {
			if snap[i].ID == "T1" {
				t1 = &snap[i]
			}
		}
		if t1 != nil {
			fmt.Printf("t+%4dms %s %s state=%s coast=%d\n",
				probe.Milliseconds(), t1.ID, t1.Status, t1.State, t1.CoastFrames)
		} else {
			fmt.Printf("t+%4dms T1 已不在快照中（判丢时刻之后不再出现）\n", probe.Milliseconds())
		}
	}

	fmt.Println("\n== 冲突取舍记录 ==")
	for _, c := range tr.Conflicts() {
		fmt.Printf("t+%dms track=%s 采用=%s 拒绝=%s (%s %.2f vs %.2f)\n  规则: %s\n",
			c.Time.Sub(t0).Milliseconds(), c.TrackID, c.Chosen, c.Rejected,
			c.Metric, c.ChosenVal, c.OtherVal, c.Rule)
	}

	fmt.Println("\n== 最终轨迹 ==")
	for _, tv := range tr.Tracks() {
		fmt.Printf("%s status=%s first=t+%dms last=t+%dms state=%s sources=%v\n",
			tv.ID, tv.Status,
			tv.FirstSeen.Sub(t0).Milliseconds(),
			tv.LastSeen.Sub(t0).Milliseconds(),
			tv.State, tv.SourceSensors)
	}

	// 重放复现：等价于重启后再灌一遍同样的数据。
	rp := tracker.Replay(cfg, simulate.Sensors(), tr.AcceptedFrames())
	same := equalTracks(tr.Tracks(), rp.Tracks())
	fmt.Printf("\n== 重放复现 ==\n两次轨迹与标识完全一致: %v\n", same)
}

func contains(xs []string, x string) bool {
	for _, v := range xs {
		if v == x {
			return true
		}
	}
	return false
}

func equalTracks(a, b []tracker.TrackView) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i].ID != b[i].ID ||
			a[i].Status != b[i].Status ||
			!a[i].FirstSeen.Equal(b[i].FirstSeen) ||
			!a[i].LastSeen.Equal(b[i].LastSeen) ||
			a[i].State != b[i].State {
			return false
		}
	}
	return true
}
