package tracker_test

import (
	"fmt"
	"testing"

	"github.com/lechandonga/multi-sensor-track-fusion-go/tracker"
)

// bizLog 让每个用例的日志都带上业务主题，便于从 go test 输出中看出
// 覆盖了哪些业务、各业务用例的通过情况。
func bizLog(t *testing.T, business, caseName, format string, args ...any) {
	t.Helper()
	t.Logf("[业务:%s][用例:%s] %s", business, caseName, fmt.Sprintf(format, args...))
}

func rejectKind(t *testing.T, err error) tracker.RejectKind {
	t.Helper()
	rj, ok := err.(*tracker.FrameRejectedError)
	if !ok {
		t.Fatalf("期望 *FrameRejectedError，实际 %T: %v", err, err)
	}
	return rj.Kind
}
