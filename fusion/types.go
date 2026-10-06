package fusion

import (
	"errors"
	"time"
)

// ObjectClass is the coarse classification of a tracked object.
type ObjectClass string

const (
	ClassUnknown ObjectClass = "unknown"
	ClassCar     ObjectClass = "car"
	ClassPed     ObjectClass = "pedestrian"
	ClassTruck   ObjectClass = "truck"
)

// Detection is a single object observation produced by one sensor at one time.
// All coordinates are expressed in a common sensor-suite frame (metres).
// Timestamps are produced by the sensor's own clock; the engine corrects a
// fixed, configured per-sensor offset before using them.
type Detection struct {
	ID          string      // unique within one sensor frame (duplicate guard)
	X, Y        float64     // position in the shared frame
	VX, VY      float64     // sensor-reported velocity; zero when HasVelocity is false
	HasVelocity bool        // whether VX/VY are meaningful
	Class       ObjectClass // sensor-reported class; ClassUnknown allowed
}

// Frame is one batch of detections a sensor produced at time SensorTime.
type Frame struct {
	SensorID   string
	SensorTime time.Time // raw, sensor-local timestamp
	Detections []Detection
}

// TrackStatus is the externally visible lifecycle state of a track.
type TrackStatus string

const (
	// StatusActive: at least one detection was associated in the latest epoch.
	StatusActive TrackStatus = "active"
	// StatusCoasting: no recent detection, but still within the loss window.
	StatusCoasting TrackStatus = "coasting"
	// StatusLost: continuously unseen past the loss window; history retained.
	StatusLost TrackStatus = "lost"
)

// State is the fused estimate at one observation time.
type State struct {
	Time   time.Time
	X, Y   float64
	VX, VY float64
	Class  ObjectClass
	// Sources records which sensors contributed to this state, in priority
	// order, so every fused value can be explained.
	Sources []SourceEvidence
}

// SourceEvidence records one sensor's contribution to a fused State.
type SourceEvidence struct {
	SensorID   string
	DetectionID string
	X, Y       float64
}

// Track is the externally visible life story of one real-world object.
// One physical object always maps to exactly one stable TrackID.
type Track struct {
	ID       int64
	Status   TrackStatus
	BornAt   time.Time
	LastSeen time.Time
	LostAt   time.Time // zero unless Status == lost
	Current  State
	// History holds one fused state per epoch the track existed in
	// (active and coasting epochs alike), in chronological order.
	History []State
}

// Snapshot is a point-in-time, immutable view of the full tracker world.
type Snapshot struct {
	At     time.Time
	Tracks []Track
}

// RejectReason categorises why a frame was refused.
type RejectReason string

const (
	// RejectDuplicate: an identical frame (sensor + corrected time) was
	// already accepted.
	RejectDuplicate RejectReason = "duplicate_frame"
	// RejectTooLate: corrected time is before the per-sensor late horizon.
	RejectTooLate RejectReason = "beyond_late_horizon"
	// RejectBadTimestamp: zero time or time after the configured future slack.
	RejectBadTimestamp RejectReason = "invalid_timestamp"
	// RejectUnknownSensor: sensor id has no configuration registered.
	RejectUnknownSensor RejectReason = "unknown_sensor"
	// RejectDupDetection: two detections with the same id inside one frame.
	RejectDupDetection RejectReason = "duplicate_detection_in_frame"
)

// RejectError is returned (or recorded) whenever a frame cannot be accepted.
type RejectError struct {
	SensorID string
	Reason   RejectReason
	Detail   string
}

func (e *RejectError) Error() string {
	return "fusion: frame rejected: sensor=" + e.SensorID +
		" reason=" + string(e.Reason) + " " + e.Detail
}

// Is lets callers use errors.Is(err, RejectTooLate) etc.
func (e *RejectError) Is(target error) bool {
	t, ok := target.(*RejectError)
	if !ok {
		return false
	}
	return t.Reason == e.Reason
}

// Sentinel values for errors.Is(err, fusion.RejectTooLate).
var (
	errSentinel = func() map[RejectReason]error {
		m := map[RejectReason]error{}
		for _, r := range []RejectReason{
			RejectDuplicate, RejectTooLate, RejectBadTimestamp,
			RejectUnknownSensor, RejectDupDetection,
		} {
			m[r] = &RejectError{Reason: r}
		}
		return m
	}()
)

func reasonErr(r RejectReason) error { return errSentinel[r] }

var _ = errors.Is // keep errors import when error helpers evolve
