// Package simulate 生成纯本地、固定种子的多传感器模拟检测数据，
// 不接任何真实设备或外部服务。
package simulate

import (
	"math/rand"
	"time"

	"github.com/lechandonga/multi-sensor-track-fusion-go/tracker"
)

// TruthPoint 是模拟世界中目标在某时刻的真实状态。
type TruthPoint struct {
	Time     time.Time
	X, Y     float64
	VX, VY   float64
	Class    tracker.Class
	Occluded bool
}

// TargetTruth 是一个目标全程真值。
type TargetTruth struct {
	Name  string
	Start time.Time
	End   time.Time
	Class tracker.Class
	At    func(time.Time) (x, y, vx, vy float64)
	// Occluded 声明该目标在 [from,to) 内所有传感器均丢检（遮挡）。
	Occluded func(time.Time) bool
}

// DeliveryItem 是一次"送达"动作：Idx 指向 Scenario.Frames 的下标。
type DeliveryItem struct {
	Idx int
	// Arrival 仅用于展示与解释（决定送达先后），不参与融合。
	Arrival   time.Time
	Duplicate bool
}

// Scenario 是一次完整模拟的产物。
type Scenario struct {
	T0       time.Time
	Frames   []tracker.Frame
	Delivery []DeliveryItem
	Truth    []TargetTruth
	// ConflictAt 标注注入的传感器冲突时刻。
	ConflictAt time.Time
	// 各类说明，供测试与文档引用。
	DuplicateOf map[int]int // 送达下标 -> 原帧下标
}

// Options 控制模拟内容。
type Options struct {
	Duration   time.Duration
	Step       time.Duration
	Seed       int64
	Shuffle    bool // 制造乱序送达（仍保证在迟到窗口内）
	Duplicates bool // 制造重复帧送达
	Conflict   bool // 制造一次相机冲突观测
}

// DefaultOptions 返回演示/测试共用的默认场景参数。
func DefaultOptions() Options {
	return Options{
		Duration:   4 * time.Second,
		Step:       100 * time.Millisecond,
		Seed:       20261007,
		Shuffle:    true,
		Duplicates: true,
		Conflict:   true,
	}
}

type sensorSpec struct {
	sensor tracker.Sensor
	noise  float64
	vel    bool
	// detectClass 该传感器给出的类别（空表示 unknown）。
	detectClass tracker.Class
	// missProb 独立漏检概率（遮挡之外的随机丢检）。
	missProb float64
}

// Sensors 返回与模拟数据匹配的传感器注册集合。
func Sensors() []tracker.Sensor {
	return []tracker.Sensor{
		{ID: "lidar", Kind: tracker.SensorLidar, ClockOffset: -20 * time.Millisecond, Priority: 0},
		{ID: "radar", Kind: tracker.SensorRadar, ClockOffset: 30 * time.Millisecond, Priority: 1},
		{ID: "camera", Kind: tracker.SensorCamera, ClockOffset: 10 * time.Millisecond, Priority: 2},
	}
}

func sensorSpecs() []sensorSpec {
	return []sensorSpec{
		{
			sensor:      tracker.Sensor{ID: "lidar", Kind: tracker.SensorLidar, ClockOffset: -20 * time.Millisecond, Priority: 0},
			noise:       0.15,
			detectClass: tracker.ClassUnknown,
		},
		{
			sensor:      tracker.Sensor{ID: "radar", Kind: tracker.SensorRadar, ClockOffset: 30 * time.Millisecond, Priority: 1},
			noise:       0.12,
			vel:         true,
			detectClass: tracker.ClassCar,
			missProb:    0.05,
		},
		{
			sensor:      tracker.Sensor{ID: "camera", Kind: tracker.SensorCamera, ClockOffset: 10 * time.Millisecond, Priority: 2},
			noise:       0.25,
			detectClass: tracker.ClassCar,
			missProb:    0.10,
		},
	}
}

func specByID(id string) sensorSpec {
	for _, sp := range sensorSpecs() {
		if sp.sensor.ID == id {
			return sp
		}
	}
	return sensorSpec{}
}

// Generate 按固定规则生成场景。同一 Options 多次调用结果逐字节一致。
func Generate(t0 time.Time, opt Options) *Scenario {
	rng := rand.New(rand.NewSource(opt.Seed))
	specs := sensorSpecs()

	targetA := TargetTruth{
		Name:  "A",
		Start: t0,
		End:   t0.Add(opt.Duration),
		Class: tracker.ClassCar,
		At: func(t time.Time) (float64, float64, float64, float64) {
			s := t.Sub(t0).Seconds()
			return 10.0 + 4.0*s, 5.0, 4.0, 0.0
		},
		// 1.2s~1.7s 被遮挡：三路同时丢检，用于验证 coasting 与续接。
		Occluded: func(t time.Time) bool {
			s := t.Sub(t0).Seconds()
			return s >= 1.2 && s < 1.7
		},
	}
	targetB := TargetTruth{
		Name:  "B",
		Start: t0.Add(2500 * time.Millisecond),
		End:   t0.Add(opt.Duration),
		Class: tracker.ClassTruck,
		At: func(t time.Time) (float64, float64, float64, float64) {
			s := t.Sub(t0).Seconds() - 2.5
			return 2.0 - 2.0*s, -8.0, -2.0, 0.0
		},
	}
	truth := []TargetTruth{targetA, targetB}

	conflictAt := t0.Add(800 * time.Millisecond)

	type genFrame struct {
		f       tracker.Frame
		arrival time.Time
	}
	var gens []genFrame

	steps := int(opt.Duration / opt.Step)
	for _, sp := range specs {
		for k := 0; k < steps; k++ {
			corrected := t0.Add(time.Duration(k) * opt.Step)
			raw := corrected.Add(-sp.sensor.ClockOffset)
			arrival := corrected.Add(time.Duration(10+rng.Intn(40)) * time.Millisecond)

			var dets []tracker.Detection
			for ti, tg := range truth {
				if corrected.Before(tg.Start) || !corrected.Before(tg.End) {
					continue
				}
				x, y, vx, vy := tg.At(corrected)
				if tg.Occluded != nil && tg.Occluded(corrected) {
					continue
				}
				if rng.Float64() < sp.missProb {
					continue
				}
				d := tracker.Detection{
					ID:    "",
					X:     x + rng.NormFloat64()*sp.noise,
					Y:     y + rng.NormFloat64()*sp.noise,
					Class: sp.detectClass,
				}
				if sp.vel {
					d.VX = vx + rng.NormFloat64()*0.1
					d.VY = vy + rng.NormFloat64()*0.1
					d.HasVelocity = true
				}
				// 相机在冲突时刻对目标 A 给出位置严重冲突、类别也打架的观测。
				if opt.Conflict && sp.sensor.ID == "camera" && corrected.Equal(conflictAt) && ti == 0 {
					d.X += 3.0
					d.Y += 1.0
					d.Class = tracker.ClassPed
				}
				dets = append(dets, d)
			}

			gens = append(gens, genFrame{
				f: tracker.Frame{
					SensorID:   sp.sensor.ID,
					Seq:        uint64(k + 1),
					RawTime:    raw,
					Detections: dets,
				},
				arrival: arrival,
			})
		}
	}

	// 送达顺序：每个时间步内三路帧按确定性的随机传感器排列送达，
	// 同一路传感器的帧在送达序列中始终严格有序（不会自乱序号），
	// 但跨传感器的相对先后是乱序的，且全部落在迟到窗口之内。
	var order []int
	byStepSensor := map[int]map[string]int{}
	for gi, g := range gens {
		step := int(g.f.RawTime.Sub(t0.Add(-specByID(g.f.SensorID).sensor.ClockOffset)) / opt.Step)
		if byStepSensor[step] == nil {
			byStepSensor[step] = map[string]int{}
		}
		byStepSensor[step][g.f.SensorID] = gi
	}
	for k := 0; k < steps; k++ {
		ids := []string{"lidar", "radar", "camera"}
		if opt.Shuffle {
			rng.Shuffle(len(ids), func(i, j int) { ids[i], ids[j] = ids[j], ids[i] })
		}
		for _, id := range ids {
			if gi, ok := byStepSensor[k][id]; ok {
				order = append(order, gi)
			}
		}
	}

	sc := &Scenario{
		T0:          t0,
		Truth:       truth,
		ConflictAt:  conflictAt,
		DuplicateOf: map[int]int{},
	}
	frameAt := make(map[int]int) // gen 下标 -> Frames 下标
	for _, gi := range order {
		frameAt[gi] = len(sc.Frames)
		sc.Frames = append(sc.Frames, gens[gi].f)
		sc.Delivery = append(sc.Delivery, DeliveryItem{
			Idx: len(sc.Frames) - 1, Arrival: gens[gi].arrival,
		})
	}

	if opt.Duplicates {
		// 选雷达第 3 帧原样再送一次：必须被识别为重复帧而不是二次累计。
		for gi, g := range gens {
			if g.f.SensorID == "radar" && g.f.Seq == 3 {
				dup := g.f
				sc.DuplicateOf[len(sc.Frames)] = frameAt[gi]
				sc.Frames = append(sc.Frames, dup)
				sc.Delivery = append(sc.Delivery, DeliveryItem{
					Idx:       len(sc.Frames) - 1,
					Arrival:   g.arrival.Add(120 * time.Millisecond),
					Duplicate: true,
				})
				break
			}
		}
	}

	return sc
}

// TruthAt 返回目标在指定时刻的真值位置（不存在返回 false）。
func TruthAt(tg TargetTruth, t time.Time) (x, y, vx, vy float64, ok bool) {
	if t.Before(tg.Start) || t.After(tg.End) {
		return 0, 0, 0, 0, false
	}
	x, y, vx, vy = tg.At(t)
	return
}
