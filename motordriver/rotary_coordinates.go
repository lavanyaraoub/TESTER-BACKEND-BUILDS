package motordriver

import (
	"fmt"
	"time"
)

// MotionTarget separates a fixed destination from a relative wire command.
// Delta includes intentional full turns; Position is the expected wrapped end.
// The zero value is an ordinary absolute target at zero.
type MotionTarget struct {
	Position int32
	Start    int32
	Delta    int64
	Period   int64 // zero for the existing absolute PP path
}

// PPCommand is immutable once published to the PDO goroutine.
type PPCommand struct {
	Value    int32 // 607A payload: a distance, not the expected end position
	Expected int32
	// Reference is the actual position used when this command was prepared.
	// The cyclic task rebases a fixed endpoint again just before bit4 rises.
	Reference    int32
	LateBound    bool
	DistanceOnly bool // G91 retains its distance; G90 retains its destination
}

func wrapRotary(value, period int64) int64 {
	value %= period
	if value < 0 {
		value += period
	}
	return value
}

// rotaryDifference is for endpoint comparisons and small pre-latch drift only.
// Never apply it to a requested distance: doing so would discard full turns.
func rotaryDifference(target, actual, period int64) int64 {
	delta := wrapRotary(target-actual, period)
	if delta > period/2 {
		delta -= period
	}
	return delta
}

func makeMotionTarget(goal int64, start int32, period int64) (MotionTarget, error) {
	if period == 0 {
		if goal < -1<<31 || goal > 1<<31-1 {
			return MotionTarget{}, fmt.Errorf("absolute target overflow: goal=%d", goal)
		}
		return MotionTarget{Position: int32(goal)}, nil
	}
	if period < 2 || period > 1<<31-1 || int64(start) < 0 || int64(start) >= period {
		return MotionTarget{}, fmt.Errorf("invalid rotary coordinate: actual=%d period=%d", start, period)
	}
	// The useful goal lies within one signed 32-bit distance of the start.
	// Check before subtraction so malformed goals cannot overflow int64.
	if goal < int64(start)-(1<<31-1) || goal > int64(start)+(1<<31-1) {
		return MotionTarget{}, fmt.Errorf("relative distance exceeds int32; split the move: goal=%d start=%d", goal, start)
	}
	return MotionTarget{Position: int32(wrapRotary(goal, period)), Start: start,
		Delta: goal - int64(start), Period: period}, nil
}

func (target MotionTarget) relativeCommand(actual int32, distanceOnly bool) (PPCommand, error) {
	if target.Period < 2 || int64(actual) < 0 || int64(actual) >= target.Period {
		return PPCommand{}, fmt.Errorf("actual position %d outside continuous range", actual)
	}
	delta := target.Delta
	expected := target.Position
	if distanceOnly {
		// G91 means this distance from the live, stopped position.
		expected = int32(wrapRotary(int64(actual)+delta, target.Period))
	} else {
		// G90 and Zero Reference keep their original fixed destination. Correct
		// for any small movement while waiting for the clamp to release.
		delta -= rotaryDifference(int64(actual), int64(target.Start), target.Period)
	}
	if delta < -1<<31 || delta > 1<<31-1 {
		return PPCommand{}, fmt.Errorf("relative distance %d exceeds int32; split the move", delta)
	}
	return PPCommand{Value: int32(delta), Expected: expected, Reference: actual, LateBound: true, DistanceOnly: distanceOnly}, nil
}

// ContinuousGeometry is read before activation. No live SDO parameter writes
// are introduced by this feature. These limits initially describe A6B mode 4.
type ContinuousGeometry struct {
	Mode, Upper, Feed, FeedTurns, MotorTurns, ShaftTurns int64
	Encoder, EncoderTurns, Minimum, Maximum, Offset      int64
	RelativeOption, Polarity, LimitMin, LimitMax         int64
}

func (g ContinuousGeometry) validate(ratio, motorTurnsPerTableRev int) (int64, string, error) {
	if motorTurnsPerTableRev <= 0 || g.Upper+1 != int64(motorTurnsPerTableRev) || g.Mode != 4 || g.Upper < 1 || g.Upper > 65534 || g.Feed <= 0 ||
		g.FeedTurns != 1 || g.MotorTurns != 1 || g.ShaftTurns != 1 ||
		g.Encoder != 8388608 || g.EncoderTurns != 1 || g.RelativeOption != 2 ||
		g.LimitMin != g.LimitMax || ratio <= 0 || ratio > (1<<31-1)/360 {
		return 0, "", fmt.Errorf("unsupported continuous geometry: %+v; require matching motor-turns-per-table-rev, mode 4, 23-bit encoder, 6091=1:1, 6092:02=1, 60F2=2 and disabled software limits", g)
	}
	period := int64(ratio) * 360
	if g.Feed*(g.Upper+1) != period || g.Minimum != 0 || g.Maximum != period-1 ||
		g.Offset < 0 || g.Offset >= period {
		return 0, "", fmt.Errorf("continuous period mismatch: software=%d feed=%d upper=%d range=[%d,%d] offset=%d", period, g.Feed, g.Upper, g.Minimum, g.Maximum, g.Offset)
	}
	// The fingerprint deliberately excludes the current position. A changed
	// drive scaling, period, polarity or offset invalidates the saved home.
	return period, fmt.Sprintf("a6b-mode4:%d:%+v", ratio, g), nil
}

// Position spread used to distinguish settled feedback from merely passing
// through the application's acceptance band. No drive parameter is written.
const (
	positionStabilityPulses = int64(5)
	positionLatchQuietTime  = 10 * time.Millisecond
	positionLatchMaxGap     = 3 * time.Millisecond
)

type positionStability struct {
	started, last    time.Time
	anchor           int32
	minimum, maximum int64
}

func (s *positionStability) reset() { *s = positionStability{} }

// observe uses wrapped differences so a rotary rollover cannot look like a
// large jump. A scheduling gap is not evidence of continuously quiet feedback.
func (s *positionStability) observe(actual int32, period int64, now time.Time, window, maxGap time.Duration) bool {
	if s.started.IsZero() || now.Sub(s.last) > maxGap {
		*s = positionStability{started: now, last: now, anchor: actual}
		return false
	}
	delta := int64(actual) - int64(s.anchor)
	if period > 0 {
		delta = rotaryDifference(int64(actual), int64(s.anchor), period)
	}
	if delta < s.minimum {
		s.minimum = delta
	}
	if delta > s.maximum {
		s.maximum = delta
	}
	if s.maximum-s.minimum > positionStabilityPulses {
		*s = positionStability{started: now, last: now, anchor: actual}
		return false
	}
	s.last = now
	return now.Sub(s.started) >= window
}

// atLatch preserves intentional full turns. Only small coordinate drift is
// wrapped; the requested distance itself is never reduced modulo one turn.
func (command PPCommand) atLatch(actual int32, period int64) (PPCommand, error) {
	if period < 2 || period > 1<<31-1 || actual < 0 || int64(actual) >= period ||
		command.Reference < 0 || int64(command.Reference) >= period ||
		command.Expected < 0 || int64(command.Expected) >= period {
		return PPCommand{}, fmt.Errorf("invalid relative latch coordinate: reference=%d actual=%d expected=%d period=%d", command.Reference, actual, command.Expected, period)
	}
	if !command.LateBound {
		return command, nil
	}
	if !command.DistanceOnly && int32(wrapRotary(int64(command.Reference)+int64(command.Value), period)) != command.Expected {
		return PPCommand{}, fmt.Errorf("inconsistent fixed endpoint: reference=%d delta=%d expected=%d", command.Reference, command.Value, command.Expected)
	}
	delta := int64(command.Value)
	expected := command.Expected
	if command.DistanceOnly {
		expected = int32(wrapRotary(int64(actual)+delta, period))
	} else {
		delta -= rotaryDifference(int64(actual), int64(command.Reference), period)
	}
	if delta < -1<<31 || delta > 1<<31-1 {
		return PPCommand{}, fmt.Errorf("relative distance at latch exceeds int32: %d", delta)
	}
	// A late rebase must never introduce an unplanned reversal. Such a
	// change needs a fresh direction/limit plan rather than an automatic move.
	if command.Value != 0 && delta != 0 && (command.Value < 0) != (delta < 0) {
		return PPCommand{}, fmt.Errorf("relative latch would reverse the planned direction: prepared=%d latched=%d", command.Value, delta)
	}
	return PPCommand{Value: int32(delta), Expected: expected, Reference: actual, LateBound: command.LateBound, DistanceOnly: command.DistanceOnly}, nil
}

type ppCommandError struct{ Message string }

type ppLatchRecord struct {
	PreparedReference, Actual, Value, Expected int32
	Waited                                     time.Duration
	TriggeredAt                                time.Time
}

type ppLatchOutput struct {
	Command                 PPCommand
	Controlword             uint16
	Triggered, Acknowledged bool
}

// ppRelativeLatch is owned exclusively by one axis of the PDO cyclic task.
// It stages one payload with bit4 LOW, then freezes it from the rising edge
// through acknowledgement. No retries, sleeps, logging or SDO calls occur here.
type ppRelativeLatch struct {
	source                          *PPCommand
	quiet                           positionStability
	firstSeen                       time.Time
	staged, triggered, acknowledged bool
	wire                            PPCommand
}

func (s *ppRelativeLatch) reset() { *s = ppRelativeLatch{} }

// holdOnInvalidFeedback never creates a new edge. Once triggered, retain the
// HIGH until a valid ACK arrives; before triggering require fresh staging again.
func (s *ppRelativeLatch) holdOnInvalidFeedback(command *PPCommand) ppLatchOutput {
	out := ppLatchOutput{Command: *command, Controlword: 0x004F}
	if s.source == command {
		out.Command = s.wire
		if s.triggered && !s.acknowledged {
			out.Controlword |= 0x0010
		}
	}
	if !s.triggered {
		s.quiet.reset()
		s.staged = false
	}
	return out
}

func (s *ppRelativeLatch) step(command *PPCommand, actual int32, status uint16, modeReady bool, period int64, now time.Time) (ppLatchOutput, error) {
	const base = uint16(0x004F) // Operation Enabled, relative PP, new-setpoint LOW
	if s.source != command {
		*s = ppRelativeLatch{source: command, firstSeen: now, wire: *command}
	}
	out := ppLatchOutput{Command: s.wire, Controlword: base}
	if s.triggered {
		// Preserve exactly the same distance while the drive acknowledges it.
		if s.acknowledged || status&0x1000 != 0 {
			s.acknowledged = true
			out.Acknowledged = true
		} else {
			out.Controlword |= 0x0010
		}
		return out, nil
	}
	// Wait for the previous ACK to fall and Mode 1 to become active.
	if !modeReady || status&0x1000 != 0 {
		s.quiet.reset()
		s.staged = false
		return out, nil
	}
	if !s.quiet.observe(actual, period, now, positionLatchQuietTime, positionLatchMaxGap) {
		s.staged = false
		return out, nil
	}
	prepared, err := command.atLatch(actual, period)
	if err != nil {
		return out, err
	}
	if !s.staged || prepared.Value != s.wire.Value || prepared.Expected != s.wire.Expected {
		// Deliver the freshly computed distance for one PDO frame before
		// raising bit4. Re-evaluate from fresh feedback on the next frame.
		s.wire = prepared
		out.Command = prepared
		s.staged = true
		return out, nil
	}
	// The same payload was delivered with bit4 LOW in the previous valid
	// cycle. Preserve its reference/value/expected tuple through ACK.
	out.Command = s.wire
	s.triggered = true
	out.Triggered = true
	out.Controlword |= 0x0010
	return out, nil
}
