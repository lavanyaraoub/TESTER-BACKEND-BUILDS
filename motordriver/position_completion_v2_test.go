package motordriver

import (
	"testing"
	"time"
)

func TestV2DriveReachedAcceptsNormalInWindowEncoderJitter(t *testing.T) {
	start := time.Unix(1700000000, 0)
	c := newPositionCompletion(35489, 7200000, start)
	jitter := []int32{0, 6, 0, -6, 0, 5, 0, -5}
	for step := 0; step <= 450; step++ {
		actual := int32(35489) + jitter[step%len(jitter)]
		s := sampleAt(start, step, actual, 35489-actual)
		if step > 0 {
			s.Status |= 1 << 10
		}
		done, err := c.observe(s, s.CapturedAt, true, step > 0)
		if err != nil {
			t.Fatalf("normal drive-window jitter timed out: %v", err)
		}
		if done {
			if step < 100 {
				t.Fatal("accepted before endpoint dwell")
			}
			return
		}
	}
	t.Fatal("drive-completed endpoint was never accepted")
}

func TestV2ShortInvalidExchangesPauseInsteadOfErasingValidDwell(t *testing.T) {
	start := time.Unix(1700000000, 0)
	c := newPositionCompletion(35489, 7200000, start)
	var lastInvalid time.Time
	for step := 0; step <= 450; step++ {
		s := sampleAt(start, step, 35489, 0)
		if step > 0 {
			s.Status |= 1 << 10
		}
		// One incomplete 1ms exchange every 100ms, between observer polls.
		if step > 0 && step%20 == 0 {
			lastInvalid = s.CapturedAt.Add(-time.Millisecond)
		}
		s.LastInvalidAt = lastInvalid
		done, err := c.observe(s, s.CapturedAt, true, step > 0)
		if err != nil {
			t.Fatalf("brief feedback gaps caused a false endpoint timeout: %v", err)
		}
		if done {
			if step <= 100 {
				t.Fatal("unobserved intervals were counted as valid dwell")
			}
			return
		}
	}
	t.Fatal("valid dwell never accumulated")
}

func TestV2RepeatedRealResidualViolationsCannotBeHiddenByZeroFinalSample(t *testing.T) {
	start := time.Unix(1700000000, 0)
	c := newPositionCompletion(35489, 7200000, start)
	var lastViolation time.Time
	for step := 0; step <= 450; step++ {
		s := sampleAt(start, step, 35489, 0)
		if step > 0 {
			s.Status |= 1 << 10
		}
		if step > 0 && step%20 == 0 {
			lastViolation = s.CapturedAt.Add(-time.Millisecond)
		}
		s.LastEndpointViolationAt = lastViolation
		done, err := c.observe(s, s.CapturedAt, true, step > 0)
		if done {
			t.Fatal("accepted despite recurring out-of-25 events")
		}
		if err != nil {
			return
		}
	}
	t.Fatal("recurring endpoint violations did not fail")
}

func TestV2LongFeedbackGapsCannotAccumulateDwell(t *testing.T) {
	start := time.Unix(1700000000, 0)
	c := newPositionCompletion(35489, 7200000, start)
	for step := 0; step <= 450; step += 5 {
		s := sampleAt(start, step, 35489, 0)
		if step > 0 {
			s.Status |= 1 << 10
		}
		done, err := c.observe(s, s.CapturedAt, true, step > 0)
		if done {
			t.Fatal("accepted across repeated 25ms gaps")
		}
		if err != nil {
			if c.longGaps == 0 {
				t.Fatal("missing gap diagnostics")
			}
			return
		}
	}
	t.Fatal("never rejected long observation gaps")
}
