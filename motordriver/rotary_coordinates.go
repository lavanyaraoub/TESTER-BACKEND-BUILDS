package motordriver

import "fmt"

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
	return PPCommand{Value: int32(delta), Expected: expected}, nil
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
