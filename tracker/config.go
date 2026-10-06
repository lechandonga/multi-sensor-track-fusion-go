package tracker

import "time"

// Config 是融合器的全部可配置项。零值不可直接使用，请用 DefaultConfig。
type Config struct {
	// MaxLateArrival 帧在统一时钟下允许的最大迟到量；早于
	// 已关闭水位的帧按 RejectTooLate 明确拒绝。
	MaxLateArrival time.Duration
	// MaxFutureSkew 帧时间戳允许超前当前已知最大时间的上限，超出拒绝。
	MaxFutureSkew time.Duration
	// GateRadius 关联门限（标准化马氏距离半径）。
	GateRadius float64
	// ConflictRadius 残差超过该标准化距离时判定为"冲突观测"，
	// 不做加权混合，直接按权威规则取舍。
	ConflictRadius float64
	// PositionSigma 位置观测噪声标准差（米），门限与权重共用。
	PositionSigma float64
	// VelocitySigma 速度观测噪声标准差（米/秒）。
	VelocitySigma float64
	// ProcessSigma 过程噪声（米/秒^1.5 量级），用于预测协方差膨胀。
	ProcessSigma float64
	// ConfirmHits 命中达到该次数才由候选转为确认。
	ConfirmHits int
	// CoastTime 确认目标允许的连续丢检时长；期间为 coasting 并匀速外推。
	CoastTime time.Duration
	// TentativeTTL 候选目标在该时长内未达到确认命中数则丢弃（不判丢、不对外）。
	TentativeTTL time.Duration
	// ReacquireTime 判丢后的续接窗口：窗口内被重新看到则沿用原 ID，
	// 超过窗口（或同时刻被另一目标占据）则作为新目标并分配新 ID。
	ReacquireTime time.Duration
	// DeadHistoryTime 丢失轨迹在内存中保留用于解释/续接的时长。
	DeadHistoryTime time.Duration
	// FusionAlpha 无冲突时新观测对状态的 EMA 融合系数（0~1）。
	FusionAlpha float64
	// TrackIDPrefix 生成目标 ID 的前缀。
	TrackIDPrefix string
}

// DefaultConfig 返回一套确定性的默认参数。
func DefaultConfig() Config {
	return Config{
		MaxLateArrival:  500 * time.Millisecond,
		MaxFutureSkew:   2 * time.Second,
		GateRadius:      8.0,
		ConflictRadius:  3.0,
		PositionSigma:   0.5,
		VelocitySigma:   1.0,
		ProcessSigma:    0.3,
		ConfirmHits:     2,
		CoastTime:       400 * time.Millisecond,
		TentativeTTL:    1 * time.Second,
		ReacquireTime:   2 * time.Second,
		DeadHistoryTime: 10 * time.Second,
		FusionAlpha:     0.6,
		TrackIDPrefix:   "T",
	}
}
