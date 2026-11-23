//go:build unit

package motordriver

import (
	"EtherCAT/settings"
	"testing"
)

func TestContinuousRealPlannerPreservesDirection(t *testing.T) {
	d := makeDevice(20000)
	d.Device.ContinuousRotation = true
	d.continuousPeriod = 7200000
	settings.SetDriverSettings("A", settings.DriverSettings{HomingValid: true, HomingApos: 30000, HomingDriveXRatio: 20000})
	d.PDOPos.Store(7030000) // home + 350 degrees
	goal, err := calculateFixedAbsoluteGoal(d, 10, 20, 0, 0)
	if err != nil {
		t.Fatal(err)
	}
	if goal.Position != 230000 || goal.Delta != 400000 || goal.Period != 7200000 {
		t.Fatalf("CW: %+v", goal)
	}
	d.PDOPos.Store(230000)
	goal, err = calculateFixedAbsoluteGoal(d, 350, -20, 0, 0)
	if err != nil {
		t.Fatal(err)
	}
	if goal.Position != 7030000 || goal.Delta != -400000 {
		t.Fatalf("CCW: %+v", goal)
	}
}

func TestContinuousRelativeSetterAndAbsoluteHold(t *testing.T) {
	d := makeDevice(20000)
	d.Device.ContinuousRotation = true
	d.continuousPeriod = 7200000
	d.homingGeometry = "verified"
	d.PdoPosReady = true
	settings.SetDriverSettings("A", settings.DriverSettings{HomingValid: true, HomingDriveXRatio: 20000, HomingGeometry: "verified"})
	c := PPCommand{Value: 400000, Expected: 200000}
	if err := d.SetRelativeMovePDO(c); err != nil {
		t.Fatal(err)
	}
	if got := d.ppRelativeCommand.Load(); got == nil || *got != c || d.desiredTargetPosition.Load() != 200000 {
		t.Fatal("payload and endpoint were mixed")
	}
	d.EnablePosPDO(true)
	if d.ppRelativeCommand.Load() == nil {
		t.Fatal("relative type cleared before completion")
	}
	if err := d.SetRelativeMovePDO(c); err == nil {
		t.Fatal("active command overwritten")
	}
	d.EnablePosPDO(false)
	if err := d.SetTargetPositionPDO(123); err != nil {
		t.Fatal(err)
	}
	if d.ppRelativeCommand.Load() != nil || d.desiredTargetPosition.Load() != 123 {
		t.Fatal("hold inherited relative semantics")
	}
}
