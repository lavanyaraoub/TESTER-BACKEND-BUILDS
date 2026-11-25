package motordriver

import (
	"errors"
	"strconv"
	"sync"
	"testing"
	"time"
)

func sampleAt(start time.Time, step int, position, following int32) PDOFeedbackSnapshot {
	at := start.Add(time.Duration(step) * 5 * time.Millisecond)
	return PDOFeedbackSnapshot{Sequence: uint64(step+1) * 2, CapturedAt: at, LastValidAt: at,
		Valid: true, Position: position, FollowingError: following, FollowingErrorReady: true,
		Status: 0x0237, Mode: 1, WorkingCounter: 3}
}

func TestCompletionToleranceBoundaryAndDriveReachedDoesNotWaiveAccuracy(t *testing.T) {
	for _, residual := range []int32{-109, -26, -25, 0, 25, 26, 109} {
		t.Run(strconv.Itoa(int(residual)), func(t *testing.T) {
			start := time.Unix(1700000000, 0)
			c := newPositionCompletion(3635489, 7200000, start)
			var accepted bool
			var err error
			for step := 0; step <= 650; step++ {
				s := sampleAt(start, step, 3635489+residual, 0)
				if step > 0 {
					s.Status |= 1 << 10
				}
				accepted, err = c.observe(s, s.CapturedAt, true, step > 0)
				if accepted || err != nil {
					break
				}
			}
			inside := residual >= -25 && residual <= 25
			if inside && (!accepted || err != nil) {
				t.Fatalf("residual=%d accepted=%t err=%v", residual, accepted, err)
			}
			if !inside && (accepted || !errors.Is(err, errPositionSettleTimeout)) {
				t.Fatalf("out of tolerance %d accepted=%t err=%v", residual, accepted, err)
			}
		})
	}
}

func TestCompletionRejectsSuppliedSlowDriftTrace(t *testing.T) {
	start := time.Unix(1700000000, 0)
	c := newPositionCompletion(35489, 7200000, start)
	var finalErr error
	for step := 0; step <= 900; step++ {
		ms := step * 5
		if ms > 1000 {
			ms = 1000
		}
		actual := int32(35489 - 21 - ms*19/1000)
		s := sampleAt(start, step, actual, 35489-actual)
		accepted, err := c.observe(s, s.CapturedAt, true, false)
		if accepted {
			t.Fatalf("accepted slow 21->40-pulse drift at %dms", step*5)
		}
		if err != nil {
			finalErr = err
			break
		}
	}
	if !errors.Is(finalErr, errPositionSettleTimeout) {
		t.Fatalf("wanted residual failure, got %v", finalErr)
	}
}

func TestCompletionAllowsSettledSoftwareEndpointWithBit10Low(t *testing.T) {
	start := time.Unix(1700000000, 0)
	c := newPositionCompletion(1000, 7200000, start)
	for step := 0; step <= 120; step++ {
		actual := int32(980 + step%3)
		s := sampleAt(start, step, actual, 1000-actual)
		done, err := c.observe(s, s.CapturedAt, true, false)
		if err != nil {
			t.Fatal(err)
		}
		if done {
			if step*5 < 500 {
				t.Fatal("accepted before stability window")
			}
			return
		}
	}
	t.Fatal("settled endpoint inside 25 was not accepted")
}

func TestCompletionCannotUseRepeatedOrInvalidFeedbackAsStability(t *testing.T) {
	start := time.Unix(1700000000, 0)
	c := newPositionCompletion(1000, 7200000, start)
	s := sampleAt(start, 0, 1000, 0)
	for step := 0; step < 10; step++ {
		done, err := c.observe(s, start.Add(time.Duration(step)*5*time.Millisecond), true, true)
		if done || err != nil {
			t.Fatalf("repeated snapshot accepted=%t err=%v", done, err)
		}
	}
	if _, err := c.observe(s, start.Add(51*time.Millisecond), true, true); err == nil {
		t.Fatal("stale feedback accepted")
	}
	c = newPositionCompletion(1000, 7200000, start)
	for step := 0; step < 200; step++ {
		s = sampleAt(start, step, 1000, 0)
		if step%10 == 0 {
			s.Valid = false
		}
		// The software path must still restart quiet evidence after a gap.
		// Drive-confirmed dwell is covered separately by the V2 regressions.
		done, err := c.observe(s, s.CapturedAt, true, false)
		if done || err != nil {
			t.Fatalf("interrupted windows accepted=%t err=%v", done, err)
		}
	}
}

func TestCompletionRequiresStoppedDemandForSoftwarePath(t *testing.T) {
	start := time.Unix(1700000000, 0)
	c := newPositionCompletion(1000, 7200000, start)
	for step := 0; step < 250; step++ {
		s := sampleAt(start, step, 1000, int32(step*10))
		done, err := c.observe(s, s.CapturedAt, true, false)
		if done || err != nil {
			t.Fatalf("moving demand accepted=%t err=%v", done, err)
		}
	}
	c = newPositionCompletion(1000, 7200000, start)
	for step := 0; step < 250; step++ {
		s := sampleAt(start, step, 1000, 0)
		s.FollowingErrorReady = false
		done, err := c.observe(s, s.CapturedAt, true, false)
		if done || err != nil {
			t.Fatalf("unmapped demand accepted=%t err=%v", done, err)
		}
	}
}

func TestCompletionRejectsStaleTargetReachedFromPreviousMove(t *testing.T) {
	start := time.Unix(1700000000, 0)
	c := newPositionCompletion(3600000, 7200000, start)
	for step := 0; step < 600; step++ {
		s := sampleAt(start, step, int32(step*1000), 0)
		s.Status |= 1 << 10
		done, err := c.observe(s, s.CapturedAt, true, true)
		if done || err != nil {
			t.Fatalf("stale HIGH affected active trajectory: done=%t err=%v", done, err)
		}
	}
	if !c.endpointDeadline.IsZero() {
		t.Fatal("stale HIGH started settling deadline")
	}
}

func TestCompletionModeGapAndOverallDeadline(t *testing.T) {
	start := time.Unix(1700000000, 0)
	c := newPositionCompletion(1000, 7200000, start)
	for step := 0; step < 150; step++ {
		s := sampleAt(start, step, 1000, 0)
		done, err := c.observe(s, s.CapturedAt, false, true)
		if done || err != nil {
			t.Fatalf("wrong mode done=%t err=%v", done, err)
		}
	}
	s := sampleAt(start, 6000, 1000, 0)
	if done, err := c.observe(s, s.CapturedAt, true, true); done || err == nil {
		t.Fatal("overall deadline not enforced")
	}
}

func TestStagingRestagesChangedPayloadAndFreezesThroughACK(t *testing.T) {
	start := time.Unix(1700000000, 0)
	command := PPCommand{Value: 100, Expected: 100, Reference: 0, LateBound: true}
	var latch ppRelativeLatch
	var out ppLatchOutput
	var err error
	for ms := 0; ms <= 10; ms++ {
		out, err = latch.step(&command, 0, 0x0237, true, 7200000, start.Add(time.Duration(ms)*time.Millisecond))
		if err != nil || out.Triggered {
			t.Fatalf("preparation triggered early: %+v %v", out, err)
		}
	}
	out, err = latch.step(&command, 1, 0x0237, true, 7200000, start.Add(11*time.Millisecond))
	if err != nil || out.Triggered || out.Command.Value != 99 || out.Controlword&0x10 != 0 {
		t.Fatalf("changed payload must be staged LOW: %+v %v", out, err)
	}
	out, err = latch.step(&command, 1, 0x0237, true, 7200000, start.Add(12*time.Millisecond))
	if err != nil || !out.Triggered || out.Command.Value != 99 {
		t.Fatalf("expected one rising edge: %+v %v", out, err)
	}
	out, err = latch.step(&command, 40, 0x0237, true, 7200000, start.Add(13*time.Millisecond))
	if err != nil || out.Triggered || out.Command.Value != 99 || out.Controlword&0x10 == 0 {
		t.Fatalf("payload not frozen before ACK: %+v %v", out, err)
	}
	out, err = latch.step(&command, 50, 0x1237, true, 7200000, start.Add(14*time.Millisecond))
	if err != nil || !out.Acknowledged || out.Command.Value != 99 || out.Controlword&0x10 != 0 {
		t.Fatalf("ACK release wrong: %+v %v", out, err)
	}
	for ms := 15; ms < 30; ms++ {
		out, err = latch.step(&command, 60, 0x0237, true, 7200000, start.Add(time.Duration(ms)*time.Millisecond))
		if err != nil || out.Triggered || out.Command.Value != 99 || out.Controlword&0x10 != 0 {
			t.Fatalf("re-trigger after ACK: %+v %v", out, err)
		}
	}
}

func TestSnapshotSignedFollowingErrorAndCoherenceUnderConcurrentRead(t *testing.T) {
	var cache pdoFeedbackCache
	start := time.Unix(1700000000, 0)
	var wg sync.WaitGroup
	wg.Add(1)
	go func() {
		defer wg.Done()
		for i := int32(1); i <= 20000; i++ {
			cache.publish(PDOFeedbackSnapshot{CapturedAt: start.Add(time.Duration(i) * time.Millisecond), Valid: true,
				Position: i, FollowingError: -i, FollowingErrorReady: true, Status: uint16(i), WorkingCounter: 3})
		}
	}()
	for i := 0; i < 30000; i++ {
		s, ok := cache.load()
		if !ok {
			continue
		}
		if s.FollowingError != -s.Position || s.Status != uint16(s.Position) || s.demand(7200000) != 0 {
			t.Fatalf("torn snapshot: %+v", s)
		}
		if s.CapturedAt != s.LastValidAt || s.Sequence&1 != 0 {
			t.Fatalf("timestamp/sequence inconsistent: %+v", s)
		}
	}
	wg.Wait()
	good, _ := cache.load()
	cache.publish(PDOFeedbackSnapshot{CapturedAt: good.CapturedAt.Add(time.Millisecond), Valid: false})
	bad, _ := cache.load()
	if bad.Valid || bad.LastValidAt != good.LastValidAt {
		t.Fatal("invalid exchange changed last valid timestamp")
	}
}

func TestInvalidFeedbackPreservesRaisedEdgeAndRestagesBeforeTrigger(t *testing.T) {
	start := time.Unix(1700000000, 0)
	command := PPCommand{Value: 100, Expected: 100, Reference: 0, LateBound: true}
	var latch ppRelativeLatch
	for ms := 0; ms <= 10; ms++ {
		if _, err := latch.step(&command, 0, 0x0237, true, 7200000, start.Add(time.Duration(ms)*time.Millisecond)); err != nil {
			t.Fatal(err)
		}
	}
	if out := latch.holdOnInvalidFeedback(&command); out.Controlword&0x10 != 0 || latch.staged {
		t.Fatal("invalid pre-trigger exchange did not reset staging")
	}
	var out ppLatchOutput
	for ms := 12; ms <= 23; ms++ {
		out, _ = latch.step(&command, 0, 0x0237, true, 7200000, start.Add(time.Duration(ms)*time.Millisecond))
	}
	if !latch.triggered || out.Controlword&0x10 == 0 {
		t.Fatal("valid re-staging did not trigger")
	}
	for i := 0; i < 3; i++ {
		out = latch.holdOnInvalidFeedback(&command)
		if out.Controlword&0x10 == 0 || out.Command.Value != 100 {
			t.Fatal("lost feedback dropped active edge")
		}
	}
	out, _ = latch.step(&command, 30, 0x1237, true, 7200000, start.Add(27*time.Millisecond))
	if !out.Acknowledged || out.Controlword&0x10 != 0 {
		t.Fatal("valid ACK not consumed")
	}
	if out = latch.holdOnInvalidFeedback(&command); out.Controlword&0x10 != 0 {
		t.Fatal("invalid exchange re-triggered completed command")
	}
}

func TestCompletionFaultAndDisabledStateOverrideEndpoint(t *testing.T) {
	for _, status := range []uint16{0x063F, 0x0621} {
		start := time.Unix(1700000000, 0)
		c := newPositionCompletion(1000, 7200000, start)
		s := sampleAt(start, 0, 1000, 0)
		s.Status = status
		if done, err := c.observe(s, start, true, true); done || err == nil {
			t.Fatalf("unsafe state 0x%04X accepted", status)
		}
	}
}

func TestCompletionRetainsUnobservedOneCycleViolation(t *testing.T) {
	start := time.Unix(1700000000, 0)
	for _, invalid := range []bool{false, true} {
		c := newPositionCompletion(1000, 7200000, start)
		for step := 0; step <= 205; step++ {
			s := sampleAt(start, step, 1000, 0)
			// A one-cycle event at 249ms falls between observer samples.
			if step >= 50 {
				if invalid {
					s.LastInvalidAt = start.Add(249 * time.Millisecond)
				} else {
					s.LastEndpointViolationAt = start.Add(249 * time.Millisecond)
				}
			}
			done, err := c.observe(s, s.CapturedAt, true, false)
			if err != nil {
				t.Fatal(err)
			}
			if done {
				if step < 150 {
					t.Fatalf("accepted despite unseen 1ms event: invalid=%t step=%d", invalid, step)
				}
				break
			}
			if step == 205 {
				t.Fatal("did not recover after event")
			}
		}
	}
}

func TestFullRevolutionCannotFinishWithoutObservedMovement(t *testing.T) {
	start := time.Unix(1700000000, 0)
	c := newPositionCompletion(1000, 7200000, start)
	c.armMotion(1000, 7200000)
	for step := 0; step < 1000; step++ {
		s := sampleAt(start, step, 1000, 0)
		s.Status |= 1 << 10
		if done, err := c.observe(s, s.CapturedAt, true, true); done || err != nil {
			t.Fatalf("unmoved full turn done=%t err=%v", done, err)
		}
	}
	s := sampleAt(start, 6000, 1000, 0)
	if done, err := c.observe(s, s.CapturedAt, true, true); done || err == nil {
		t.Fatal("unmoved full turn did not time out")
	}
}

func TestCompletionRequiresACKRelease(t *testing.T) {
	start := time.Unix(1700000000, 0)
	c := newPositionCompletion(1000, 7200000, start)
	for step := 0; step < 200; step++ {
		s := sampleAt(start, step, 1000, 0)
		s.Status = 0x1637
		if done, err := c.observe(s, s.CapturedAt, true, true); done || err != nil {
			t.Fatalf("ACK HIGH accepted=%t err=%v", done, err)
		}
	}
	for step := 200; step < 310; step++ {
		s := sampleAt(start, step, 1000, 0)
		s.Status = 0x0637
		done, err := c.observe(s, s.CapturedAt, true, true)
		if err != nil {
			t.Fatal(err)
		}
		if done {
			if step < 300 {
				t.Fatal("ACK release did not restart quiet window")
			}
			return
		}
	}
	t.Fatal("did not finish after ACK release")
}

func TestSimulatedTwentyFourHoursOneDirection(t *testing.T) {
	// 21,600 moves at a four-second cadence. This exercises wrapped arithmetic
	// and acceptance repeatedly; it is not a hardware endurance qualification.
	start := time.Unix(1700000000, 0)
	actual := int32(0)
	const period = int64(7200000)
	for move := int64(0); move < 21600; move++ {
		goal := int32(wrapRotary((move+1)*3600000, period))
		delta := wrapRotary(int64(goal)-int64(actual), period)
		target, err := makeMotionTarget(int64(actual)+delta, actual, period)
		if err != nil {
			t.Fatal(err)
		}
		command, err := target.relativeCommand(actual, false)
		if err != nil || command.Expected != goal || command.Value < 0 {
			t.Fatalf("move %d wrong target %+v %v", move, command, err)
		}
		at := start.Add(time.Duration(move) * 4 * time.Second)
		c := newPositionCompletion(goal, period, at)
		previous := actual
		c.armMotion(previous, int64(command.Value))
		offset := int32(move%51) - 25
		actual = int32(wrapRotary(int64(goal)+int64(offset), period))
		accepted := false
		for step := 0; step <= 105; step++ {
			s := sampleAt(at, step, actual, -offset)
			if step == 0 {
				s.Position = previous
				s.FollowingError = 0
			}
			done, err := c.observe(s, s.CapturedAt, true, false)
			if err != nil {
				t.Fatalf("move %d: %v", move, err)
			}
			if done {
				accepted = true
				break
			}
		}
		if !accepted {
			t.Fatalf("move %d did not finish inside tolerance", move)
		}
	}
}
