package motordriver

import (
	ethercatDevice "EtherCAT/ethercatdevicedatatypes"
	"EtherCAT/logger"
	"EtherCAT/motordriver/statusnotifier"
	"EtherCAT/settings"
	"fmt"
)

func motionPeriod(d *MasterDevice) int64 {
	if d.Device.ContinuousRotation {
		return d.continuousPeriod
	}
	return 0
}

func positionResidual(d *MasterDevice, target, actual int32) int64 {
	if period := motionPeriod(d); period > 0 {
		return rotaryDifference(int64(target), int64(actual), period)
	}
	return int64(target) - int64(actual)
}

// Called after parameter configuration and polarity setup, before PowerOn and
// master activation. An explicitly enabled unsupported drive fails closed.
func configureContinuousAxis(d *MasterDevice) error {
	if d.Device.DriveType != "a6_minas" {
		if d.Device.ContinuousRotation {
			return fmt.Errorf("axis %s: continuous rotation is not validated for %s", d.Name, d.Device.DriveType)
		}
		return checkSavedHomeGeometry(d)
	}
	read := func(index, sub int, dataType string) (int64, error) {
		value, err := SDOUpload(d.Master, d.Position, ethercatDevice.Step{Address: index, SubIndex: sub, DataType: dataType})
		if err != nil {
			return 0, fmt.Errorf("axis %s: read 0x%04X:%02X: %w", d.Name, index, sub, err)
		}
		if dataType == "U32" {
			return int64(uint32(value)), nil
		}
		return int64(value), nil
	}
	mode, err := read(0x3015, 0, "I16")
	if err != nil {
		return err
	}
	if !d.Device.ContinuousRotation {
		if mode == 4 {
			return fmt.Errorf("axis %s: drive is in mode 4 but continuous-rotation is false", d.Name)
		}
		return checkSavedHomeGeometry(d)
	}
	g := ContinuousGeometry{Mode: mode}
	for _, item := range []struct {
		index, sub int
		kind       string
		out        *int64
	}{
		{0x3688, 0, "I32", &g.Upper}, {0x6092, 1, "U32", &g.Feed},
		{0x6092, 2, "U32", &g.FeedTurns}, {0x6091, 1, "U32", &g.MotorTurns},
		{0x6091, 2, "U32", &g.ShaftTurns}, {0x608F, 1, "U32", &g.Encoder},
		{0x608F, 2, "U32", &g.EncoderTurns}, {0x607B, 1, "I32", &g.Minimum},
		{0x607B, 2, "I32", &g.Maximum}, {0x607C, 0, "I32", &g.Offset},
		{0x60F2, 0, "U16", &g.RelativeOption}, {0x607E, 0, "U8", &g.Polarity},
		{0x607D, 1, "I32", &g.LimitMin}, {0x607D, 2, "I32", &g.LimitMax},
	} {
		value, err := read(item.index, item.sub, item.kind)
		if err != nil {
			return err
		}
		*item.out = value
	}
	period, geometry, err := g.validate(d.Device.DriveXRatio, d.Device.MotorTurnsPerTableRev)
	if err != nil {
		return fmt.Errorf("axis %s: %w", d.Name, err)
	}
	d.continuousPeriod, d.homingGeometry = period, geometry
	logger.Info("[CONTINUOUS] validated axis=", d.Name, " period=", period, " Feed=", g.Feed, " upper=", g.Upper)
	return checkSavedHomeGeometry(d)
}

func checkSavedHomeGeometry(d *MasterDevice) error {
	ds := settings.GetDriverSettings(d.Name)
	if !ds.HomingValid {
		return nil
	}
	if ds.HomingDriveXRatio != d.Device.DriveXRatio || ds.HomingGeometry != d.homingGeometry {
		if err := settings.InvalidateHomingCalibration(d.Name); err != nil {
			return err
		}
		message := fmt.Sprintf("axis %s: home calibration no longer matches drive geometry; perform successful multiturn reset and Home Offset calibration", d.Name)
		logger.Warn(message)
		statusnotifier.Alarm(message)
	}
	return nil
}

// SetRelativeMovePDO publishes a complete immutable command before PP output is
// enabled. The expected endpoint remains separate from the transmitted delta.
func (d *MasterDevice) SetRelativeMovePDO(command PPCommand) error {
	if !d.PdoPosReady || !d.Device.ContinuousRotation || d.continuousPeriod <= 0 {
		return fmt.Errorf("axis %s: continuous PDO position mode not ready", d.Name)
	}
	if d.posMoveAborted.Load() || systemResetInProgress.Load() {
		return fmt.Errorf("axis %s: position command cancelled by stop/reset", d.Name)
	}
	if d.IsPosEnabled() {
		return fmt.Errorf("axis %s: cannot replace an active PP command", d.Name)
	}
	ds := settings.GetDriverSettings(d.Name)
	if !ds.HomingValid || ds.HomingGeometry != d.homingGeometry || ds.HomingDriveXRatio != d.Device.DriveXRatio {
		return fmt.Errorf("axis %s: continuous rotation requires a matching home calibration", d.Name)
	}
	if command.Expected < 0 || int64(command.Expected) >= d.continuousPeriod {
		return fmt.Errorf("axis %s: expected endpoint outside continuous range", d.Name)
	}
	d.desiredTargetPosition.Store(command.Expected)
	d.ppRelativeCommand.Store(&command)
	d.ppSetpointPending.Store(true)
	return nil
}
