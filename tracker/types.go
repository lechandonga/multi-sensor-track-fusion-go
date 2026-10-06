package tracker

import (
	"fmt"
	"time"
)

// Sensor 描述一路传感器的固定属性。
//
// 传感器时钟偏差采用"配置时固定声明、不做在线估计"的规则：每帧的原始
// 时间戳一律按 tCorrected = tRaw + ClockOffset 换算到统一时钟，融合结果
// 不会因为帧到达顺序不同而改变。
type Sensor struct {
	// ID 传感器唯一标识，如 "lidar"、"radar"、"camera"。
	ID string
	// Kind 类型（仅用于展示与审计）。
	Kind SensorKind
	// ClockOffset 该传感器时钟相对统一时钟的固定偏差（统一时间 = 原始时间 + ClockOffset）。
	ClockOffset time.Duration
	// Priority 权威优先级，数值越小越权威，冲突取舍时使用。
	Priority int
}

// SensorKind 传感器类型。
type SensorKind string

const (
	SensorLidar  SensorKind = "lidar"
	SensorRadar  SensorKind = "radar"
	SensorCamera SensorKind = "camera"
)

// Detection 是单条检测：某个传感器在某一时刻看到的某个候选目标。
type Detection struct {
	// ID 传感器内部的检测编号，仅用于审计与解释。
	ID string
	// X/Y 位置，单位米，统一到公共坐标系。
	X, Y float64
	// VX/VY 速度，单位米/秒；传感器给不出速度时留 0 并置 HasVelocity=false。
	VX, VY      float64
	HasVelocity bool
	// Class 目标类别；无法判定时为 ClassUnknown。
	Class Class
}

// Class 目标类别。
type Class string

const (
	ClassUnknown Class = "unknown"
	ClassCar     Class = "car"
	ClassTruck   Class = "truck"
	ClassPed     Class = "pedestrian"
)

// Frame 是一路传感器一次推送的一整帧检测。
// Seq 为该传感器帧的单调递增序号，用于重复帧与乱序检测。
type Frame struct {
	SensorID string
	Seq      uint64
	// RawTime 传感器自身时钟上的时间戳。
	RawTime    time.Time
	Detections []Detection
}

// correctedTime 返回按固定偏差修正后的统一时钟时间。
func (f Frame) correctedTime(offset time.Duration) time.Time {
	return f.RawTime.Add(offset)
}

// State 是目标在某一时刻对外暴露的融合状态（值拷贝，外部可安全持有）。
type State struct {
	X, Y   float64
	VX, VY float64
	Class  Class
}

func (s State) String() string {
	return fmt.Sprintf("(%.2f,%.2f) v=(%.2f,%.2f) class=%s", s.X, s.Y, s.VX, s.VY, s.Class)
}

// TrackStatus 目标生命周期状态。
type TrackStatus string

const (
	// StatusTentative 候选：命中次数尚不足以确认。
	StatusTentative TrackStatus = "tentative"
	// StatusConfirmed 确认目标，当前有观测支撑。
	StatusConfirmed TrackStatus = "confirmed"
	// StatusCoasting 确认目标短时丢检，靠匀速运动外推维持，尚未判丢。
	StatusCoasting TrackStatus = "coasting"
	// StatusLost 持续丢失超过阈值，已明确判丢。
	StatusLost TrackStatus = "lost"
)

// TrackView 是查询时刻对外可见的一条目标。
type TrackView struct {
	// ID 目标在整个生命周期内稳定不变的标识；消失后再续接仍沿用原 ID。
	ID     string
	Status TrackStatus
	State  State
	// FirstSeen / LastSeen 为统一时钟下的时间。
	FirstSeen time.Time
	LastSeen  time.Time
	// Hits 累计命中帧数；CoastFrames 为连续外推帧数。
	CoastFrames int
	// SourceSensors 观测过该目标的传感器集合（按首次出现顺序）。
	SourceSensors []string
}

// RejectKind 描述帧被拒绝的原因类别。
type RejectKind string

const (
	// RejectDuplicate 重复帧：同传感器同序号已接收过，幂等忽略，不重复累计。
	RejectDuplicate RejectKind = "duplicate"
	// RejectTooLate 超过允许迟到范围：修正后时间早于已关闭水位。
	RejectTooLate RejectKind = "too_late"
	// RejectFromFuture 时间戳异常：超前当前统一时钟过多。
	RejectFromFuture RejectKind = "from_future"
	// RejectBadTimestamp 时间戳异常：零值等非法时间。
	RejectBadTimestamp RejectKind = "bad_timestamp"
	// RejectOutOfOrder 同传感器序号与时间戳单调性矛盾。
	RejectOutOfOrder RejectKind = "out_of_order"
	// RejectUnknownSensor 传感器未注册。
	RejectUnknownSensor RejectKind = "unknown_sensor"
)

// FrameRejectedError 说明一帧被明确拒绝以及属于哪类问题。
type FrameRejectedError struct {
	Kind     RejectKind
	SensorID string
	Seq      uint64
	Reason   string
}

func (e *FrameRejectedError) Error() string {
	return fmt.Sprintf("tracker: frame rejected (%s) sensor=%s seq=%d: %s",
		e.Kind, e.SensorID, e.Seq, e.Reason)
}
