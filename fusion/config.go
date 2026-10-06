package fusion

import "time"

// SensorConfig describes one sensor channel.
type SensorConfig struct {
	ID string
	// ClockOffset is subtracted from the raw sensor timestamp:
	//   correctedTime = frameTime.Add(-ClockOffset)
	// A positive value means the sensor clock runs ahead of the suite clock.
	// The offset is a fixed configuration value; it is never estimated from
	// arrival order, so out-of-order delivery cannot drag the timeline.
	ClockOffset time.Duration
	// Priority breaks ties when several sensors observe one object in the
	// same fusion epoch. Lower number wins. Must be unique per sensor.
	Priority int
	// Trust is used for class conflicts: lower number wins. It normally
	// mirrors Priority but can differ (e.g. camera is best for class).
	ClassTrust int
}

// Config holds all deterministic engine parameters.
type Config struct {
	Sensors []SensorConfig
	// Epoch is the fusion grid width. Frames with corrected timestamps in
	// [k*Epoch, (k+1)*Epoch) are fused together.
	Epoch time.Duration
	// MaxLate is how long after the high-water mark a corrected frame may
	// arrive and still be incorporated (history is recomputed).
	MaxLate time.Duration
	// FutureSlack is how far ahead of "now" a timestamp may be before it is
	// rejected as an abnormal timestamp.
	FutureSlack time.Duration
	// Gate is the association gate: a detection associates to a track when
	// its distance to the predicted track state at the detection time is
	// within Gate metres.
	Gate float64
	// MaxCoast is the number of consecutive missed epochs a track is kept
	// coasting before it is declared lost.
	MaxCoast int
	// ReattachWindow: a track lost for at most this long is resumed (same
	// TrackID) when a single-sensor observation falls inside the gate from
	// its last state. After the window a reappearance always creates a new
	// track.
	ReattachWindow time.Duration
	// ConfirmHits is the number of associated detections (across any
	// sensors) needed before a track is reported as confirmed. Pending
	// tracks exist internally but are hidden from snapshots.
	ConfirmHits int
}

// DefaultConfig returns the parameters used by the local demo and tests
// unless a test overrides them.
func DefaultConfig() Config {
	return Config{
		Sensors: []SensorConfig{
			{ID: "lidar", ClockOffset: 20 * time.Millisecond, Priority: 1, ClassTrust: 2},
			{ID: "radar", ClockOffset: -10 * time.Millisecond, Priority: 2, ClassTrust: 3},
			{ID: "camera", ClockOffset: 0, Priority: 3, ClassTrust: 1},
		},
		Epoch:          100 * time.Millisecond,
		MaxLate:        500 * time.Millisecond,
		FutureSlack:    2 * time.Second,
	Gate:           3.0,
		MaxCoast:       3,
		ReattachWindow: 400 * time.Millisecond,
		ConfirmHits:    2,
	}
}
