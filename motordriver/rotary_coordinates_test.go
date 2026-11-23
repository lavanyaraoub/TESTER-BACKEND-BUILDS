package motordriver

import "testing"

func TestContinuousTargetsAcrossZero(t *testing.T) {
	const p = int64(7200000)
	for _, tc := range []struct {
		start       int32
		goal, delta int64
		end         int32
	}{
		{7000000, 7400000, 400000, 200000},
		{200000, -200000, -400000, 7000000},
		{0, p, p, 0}, {0, -p, -p, 0}, {100, p*2 + 100, p * 2, 100},
	} {
		target, err := makeMotionTarget(tc.goal, tc.start, p)
		if err != nil {
			t.Fatal(err)
		}
		command, err := target.relativeCommand(tc.start, false)
		if err != nil || int64(command.Value) != tc.delta || command.Expected != tc.end {
			t.Fatalf("start=%d goal=%d got=%+v err=%v", tc.start, tc.goal, command, err)
		}
	}
}
func TestFixedEndpointSurvivesPreLatchDrift(t *testing.T) {
	target, err := makeMotionTarget(7200100, 7199900, 7200000)
	if err != nil {
		t.Fatal(err)
	}
	// Drive drifts 102 counts forward across zero while declamping.
	c, err := target.relativeCommand(2, false)
	if err != nil || c.Value != 98 || c.Expected != 100 {
		t.Fatalf("%+v %v", c, err)
	}
	// G91 deliberately requests the complete original distance from the live start.
	c, err = target.relativeCommand(2, true)
	if err != nil || c.Value != 200 || c.Expected != 202 {
		t.Fatalf("%+v %v", c, err)
	}
}
func TestEndpointResidualAndHalfTurn(t *testing.T) {
	for _, tc := range []struct{ target, actual, want int64 }{{1, 7199999, 2}, {7199999, 1, -2}, {0, 0, 0}, {3600000, 0, 3600000}} {
		if got := rotaryDifference(tc.target, tc.actual, 7200000); got != tc.want {
			t.Fatalf("got %d want %d", got, tc.want)
		}
	}
}
func TestContinuousDistanceGuardsAndLegacyGuard(t *testing.T) {
	if _, err := makeMotionTarget(2149235507, 2145635507, 0); err == nil {
		t.Fatal("legacy overflow guard removed")
	}
	if _, err := makeMotionTarget(1<<40, 0, 7200000); err == nil {
		t.Fatal("oversized distance accepted")
	}
	if _, err := makeMotionTarget(1, 0, -1); err == nil {
		t.Fatal("invalid period accepted")
	}
	if _, err := makeMotionTarget(1, 7200000, 7200000); err == nil {
		t.Fatal("invalid raw sample accepted")
	}
}
func validContinuousGeometry() ContinuousGeometry {
	return ContinuousGeometry{Mode: 4, Upper: 119, Feed: 60000, FeedTurns: 1, MotorTurns: 1, ShaftTurns: 1,
		Encoder: 8388608, EncoderTurns: 1, Minimum: 0, Maximum: 7199999, RelativeOption: 2}
}
func TestContinuousStartupGeometry(t *testing.T) {
	g := validContinuousGeometry()
	p, stamp, err := g.validate(20000, 120)
	if err != nil || p != 7200000 || stamp == "" {
		t.Fatalf("%d %q %v", p, stamp, err)
	}
	for _, change := range []func(*ContinuousGeometry){
		func(g *ContinuousGeometry) { g.Mode = 2 }, func(g *ContinuousGeometry) { g.Upper = 89 },
		func(g *ContinuousGeometry) { g.Maximum = 7200000 }, func(g *ContinuousGeometry) { g.RelativeOption = 0 },
		func(g *ContinuousGeometry) { g.Encoder = 131072 }, func(g *ContinuousGeometry) { g.Offset = -1 },
		func(g *ContinuousGeometry) { g.LimitMax = 123 },
	} {
		bad := g
		change(&bad)
		if _, _, err := bad.validate(20000, 120); err == nil {
			t.Fatalf("accepted %+v", bad)
		}
	}
	changed := g
	changed.Polarity = 224
	_, other, _ := changed.validate(20000, 120)
	if stamp == other {
		t.Fatal("polarity change did not invalidate home fingerprint")
	}
}
func TestContinuousManyTurnsAndRestartPosition(t *testing.T) {
	// 500 physical turns at 45 degrees per request; process carries no revolution epoch.
	const p = int64(7200000)
	actual := int32(12345)
	for i := 0; i < 4000; i++ {
		goal, err := makeMotionTarget(int64(actual)+900000, actual, p)
		if err != nil {
			t.Fatal(err)
		}
		command, err := goal.relativeCommand(actual, true)
		if err != nil || command.Value != 900000 {
			t.Fatalf("cycle %d: %+v %v", i, command, err)
		}
		actual = int32(wrapRotary(int64(actual)+int64(command.Value), p))
		if actual != command.Expected {
			t.Fatal("endpoint mismatch")
		}
	}
	if actual != 12345 {
		t.Fatalf("after 500 turns: %d", actual)
	}
	// A new process can plan from the wrapped absolute feedback without RAM history.
	goal, err := makeMotionTarget(int64(actual)-900000, actual, p)
	if err != nil {
		t.Fatal(err)
	}
	c, err := goal.relativeCommand(actual, true)
	if err != nil || c.Value != -900000 {
		t.Fatalf("%+v %v", c, err)
	}
}
