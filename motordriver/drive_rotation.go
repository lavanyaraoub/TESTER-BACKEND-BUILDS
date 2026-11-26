package motordriver

import (
	helper "EtherCAT/helper"
	logger "EtherCAT/logger"
	"EtherCAT/motordriver/statusnotifier"
	settings "EtherCAT/settings"
	"errors"
	"fmt"
	"time"
)

const (
	positionCorrectionAttempts  = 0
	positionCorrectionMaxPulses = int64(100)
)

// reverseDir writes the direction-reversal polarity parameter via SDO.
// Safe only before ecrt_master_activate — called exclusively from InitMaster.
func reverseDir(masterDevice *MasterDevice) error {
	if IsPDOActive() {
		logger.Warn("[PDO] reverseDir: called after activation — skipping (direction set at startup only)")
		return nil
	}
	logger.Trace("Reverse direction activated for driver", masterDevice.Name)
	operation, err := GetEtherCATOperation("reverse", masterDevice.Device.AddressConfigName)
	if err != nil {
		return err
	}
	runSDOOperation(masterDevice.Master, masterDevice.Position, operation)
	return nil
}

// nonReverseDir writes the non-reversed direction polarity parameter via SDO.
// Safe only before ecrt_master_activate — called exclusively from InitMaster.
func nonReverseDir(masterDevice *MasterDevice) error {
	if IsPDOActive() {
		logger.Warn("[PDO] nonReverseDir: called after activation — skipping (direction set at startup only)")
		return nil
	}
	logger.Trace("Non reverse direction activated for driver", masterDevice.Name)
	operation, err := GetEtherCATOperation("nonreverse", masterDevice.Device.AddressConfigName)
	if err != nil {
		return err
	}
	runSDOOperation(masterDevice.Master, masterDevice.Position, operation)
	return nil
}

// ManualJog starts jogging. Direction can be 1 (clockwise) or -1
// (counter-clockwise). jogFeedOverride is optional so legacy callers continue
// to use the saved Machine Parameters value while the new frontend can supply
// its live speedometer value for this jog only.
func ManualJog(masterDevice *MasterDevice, direction int, jogFeedOverride ...float64) error {
	if masterDevice.posMoveAborted.Load() || systemResetInProgress.Load() {
		return fmt.Errorf("[JOG] motion blocked until Reset")
	}
	driverStatus := getCurrentDriverStatus(masterDevice.Device.Name)
	if isMotionBlockedByPotNotLimit(driverStatus, direction) {
		logger.Error("pot/not exceeded, exiting from move command")
		return errors.New("pot/not exceeded, exiting from move command")
	}
	logger.Trace("manual jog, driver: ", masterDevice.Name)
	envSettings := settings.GetDriverSettings(masterDevice.Name)
	jogFeed := float64(envSettings.JogFeed)
	jogFeedSource := "MACHINE_PARAMETER"
	if len(jogFeedOverride) > 0 {
		jogFeed = jogFeedOverride[0]
		jogFeedSource = "MANUAL_UI"
		if jogFeed < 0 || jogFeed > 20 {
			return fmt.Errorf("[JOG] invalid Manual UI JogFeed=%.6f; allowed range 0.00..20.00", jogFeed)
		}
	}

	if err := FastPowerOn(masterDevice); err != nil {
		logger.Error(err)
		statusnotifier.Alarm(err.Error())
		return err
	}

	_, declampErr := hasDeclamped(masterDevice, envSettings)
	if declampErr != nil {
		logger.Error(declampErr)
		statusnotifier.Alarm(declampErr.Error())
		return declampErr
	}

	notifyDriverStatus("motor_running", "true", masterDevice)

	// Calculate target velocity. RPMConst still comes from Sumit's active-drive
	// configuration; only the per-command JogFeed source is selectable.
	rpm := direction * int(float64(masterDevice.Device.RPMConst)*jogFeed)

	if !IsPDOActive() || !masterDevice.PdoJogReady {
		return fmt.Errorf("[JOG] PDO not active or RxPDO not ready — jog is PDO-only in this build")
	}

	// Hand the velocity setpoint to the cyclic task in a single atomic write.
	// Matches old build: SetJogPDOSetpoints(0x000F, 3, rpm) — no software ramp.
	//
	// WHY NO SOFTWARE RAMP (explanation for future developers):
	//   A goroutine that steps velocity every 10ms generates a 100 Hz torque
	//   modulation — squarely in the human hearing range — perceived as a
	//   continuous beep from the motor windings during the entire ramp.
	//   The drive's own 0x6083 profile acceleration ramp handles speed-up
	//   internally and silently.  No code timer needed.
	//
	// VELOCITY UNIT NOTE (Delta ASDA-A2-E, 0x60FF):
	//   0x60FF Target Velocity uses encoder counts/second in Mode 3.
	//   With drive-x-ratio=20000 counts/rev and e.g. 600 RPM target:
	//   targetVel = 600 × 20000 / 60 = 200,000 counts/s.
	//   rpm-const in device-configuration.yml must be set so that
	//   rpm-const × JogFeed produces a value in that range.
	//   If the value logged below is < 1000, the unit is likely wrong
	//   and the motor runs in an unstable near-zero velocity region (noisy).
	logger.Info("[PDO-JOG] Sending to cyclic. targetVel=", rpm,
		" rpm-const=", masterDevice.Device.RPMConst,
		" JogFeed=", jogFeed,
		" source=", jogFeedSource,
		" drive=", masterDevice.Name)

	if err := masterDevice.SetJogPDOSetpoints(0x000F, 3, int32(rpm)); err != nil {
		notifyDriverStatus("motor_running", "false", masterDevice)
		return fmt.Errorf("[JOG] failed to set PDO setpoints: %w", err)
	}
	masterDevice.EnableJogPDO(true)

	// Commit only after the PDO jog command has been accepted and enabled. A
	// rejected jog must not change the direction used by the next position move.
	commitMovementDirection(masterDevice, planMovementDirection(driverStatus, float64(direction)))

	return nil
}

// StopJog stop jogging smoothly
func StopJog(masterDevice *MasterDevice) error {
	logger.Trace("stop jog, driver: ", masterDevice.Name)

	if !IsPDOActive() || !masterDevice.PdoJogReady {
		logger.Warn("[PDO] StopJog: PDO not active — cannot stop jog safely for driver:", masterDevice.Name)
		return fmt.Errorf("StopJog: PDO not active — jog control is PDO-only in this build")
	}

	// Zero the velocity in a single atomic store, matching the original SDO build's
	// procedure: one write to 0x60FF = 0. The drive's 0x6084 profile deceleration
	// (set at startup in configureDriver) brings it to a smooth stop internally.
	//
	// WHY NO SOFTWARE DECEL RAMP:
	//   Same reason as ManualJog — a stepped ramp at 10ms intervals produces
	//   100 Hz torque modulation that is directly audible as a beep throughout decel.
	//   The drive's internal decel ramp is smooth, silent, and hardware-timed.
	//
	// 250ms wait: matches the middle build's proven approach. Gives the drive's
	// own 0x6084 decel ramp time to bring the motor fully to rest before
	// the cyclic standby branch takes over.
	masterDevice.desiredTargetVelocity.Store(0)
	time.Sleep(250 * time.Millisecond)

	// Cut jog mode — cyclic standby (Mode 3 vel=0) takes over immediately.
	// No hardware mode switch: drive stays in Mode 3 throughout.
	masterDevice.EnableJogPDO(false)
	logger.Info("[PDO] StopJog: velocity zeroed, standby restored for driver:", masterDevice.Name)

	notifyDriverStatus("motor_running", "false", masterDevice)
	envSettings := settings.GetDriverSettings(masterDevice.Name)
	_, clampErr := hasClamped(masterDevice, envSettings)
	if clampErr != nil {
		logger.Error(clampErr)
		statusnotifier.Alarm(clampErr.Error())
		return clampErr
	}
	return nil
}

// positionFeedbackError keeps faults, stop/reset and stale PDO feedback ahead
// of every completion decision. An emergency abort is cleared only by Reset.
func positionFeedbackError(d *MasterDevice) error {
	return positionFeedbackStateError(d, false)
}

// A confirmed clamp intentionally requests servo-off in the existing brake
// sequence. Validate faults/cancellation/freshness without rejecting that state.
func positionFeedbackStateError(d *MasterDevice, clampComplete bool) error {
	if !IsPDOActive() || !d.PdoPosReady {
		return fmt.Errorf("[PDO-PP] position feedback unavailable")
	}
	if d.posMoveAborted.Load() || systemResetInProgress.Load() || (!clampComplete && !d.IsPosEnabled()) {
		return fmt.Errorf("[PDO-PP] position move cancelled by stop/reset")
	}
	if preparation := d.ppPrepareError.Load(); preparation != nil {
		return fmt.Errorf("[PDO-PP PREPARE] %s", preparation.Message)
	}
	if healthy, age := PDOHealthy(); !healthy {
		return fmt.Errorf("[PDO-PP] stale PDO heartbeat, age=%s", age)
	}
	snapshot, ok := d.ReadPDOFeedback()
	if !ok {
		// A reader can overlap the bounded cyclic publication. The caller
		// retries; heartbeat and move deadlines still bound the wait.
		return nil
	}
	if snapshot.LastValidAt.IsZero() || time.Since(snapshot.LastValidAt) > positionFeedbackMaxAge || time.Now().Before(snapshot.LastValidAt) {
		return fmt.Errorf("[PDO-PP] stale valid PDO feedback; sample=%d wkc=%d", snapshot.Sequence, snapshot.WorkingCounter)
	}
	// Never decide a state transition from an incomplete exchange.
	if !snapshot.Valid {
		return nil
	}
	sw := snapshot.Status
	if sw&0x0008 != 0 {
		return fmt.Errorf("[PDO-PP] drive fault: sw=0x%04X err=0x%04X",
			sw, snapshot.ErrorCode)
	}
	if !clampComplete && sw&0x006F != 0x0027 {
		return fmt.Errorf("[PDO-PP] drive left Operation Enabled, sw=0x%04X", sw)
	}
	return nil
}

// hasTargetReached remains a compatibility entry point for existing callers.
// Production moves use the same single bounded observer directly.
func hasTargetReached(d *MasterDevice) error {
	_, _, err := waitForPositionCompletion(d)
	return err
}

func waitForPositionCompletion(d *MasterDevice) (int32, int64, error) {
	const handshakeTimeout = 2 * time.Second
	started := time.Now()
	handshakeDeadline := started.Add(handshakeTimeout)
	var completion *positionCompletion
	var last PDOFeedbackSnapshot
	var lastObserverReport time.Time
	var initialPosition int32
	haveInitial := false
	for {
		if time.Since(started) >= positionMoveTimeout {
			if completion != nil {
				return last.Position, 0, completion.failure(last, "move deadline exceeded")
			}
			return last.Position, 0, fmt.Errorf("[PDO-PP] timeout waiting for coherent acknowledged feedback")
		}
		if err := positionFeedbackError(d); err != nil {
			return last.Position, 0, err
		}
		s, ok := d.ReadPDOFeedback()
		if !ok {
			time.Sleep(time.Millisecond)
			continue
		}
		last = s
		if s.Valid && !haveInitial {
			initialPosition = s.Position
			haveInitial = true
		}
		now := time.Now()
		if d.ppSetpointPending.Load() || (s.Valid && s.Status&0x1000 != 0) {
			deadline := handshakeDeadline
			if record := d.ppLatchRecord.Load(); record != nil {
				deadline = record.TriggeredAt.Add(handshakeTimeout)
			}
			if !now.Before(deadline) {
				return s.Position, 0, fmt.Errorf("[PDO-PP] handshake timeout: ACK did not complete HIGH/LOW cycle; sw=0x%04X sample=%d wkc=%d", s.Status, s.Sequence, s.WorkingCounter)
			}
			time.Sleep(time.Millisecond)
			continue
		}
		if completion == nil {
			c := newPositionCompletion(d.desiredTargetPosition.Load(), motionPeriod(d), started)
			completion = &c
			if record := d.ppLatchRecord.Load(); record != nil {
				completion.armMotion(record.Actual, int64(record.Value))
				logPDODiagnostic("[PP-LATCH] drive=", d.Name, " preparedReference=", record.PreparedReference,
					" actualAtTrigger=", record.Actual, " wireDelta=", record.Value,
					" expected=", record.Expected, " prepareTime=", record.Waited)
			} else {
				completion.armMotion(initialPosition, int64(completion.goal)-int64(initialPosition))
			}
		}
		modeReady := !d.PdoOpModeDisplayReady || s.Mode == 1
		done, err := completion.observe(s, now, modeReady, d.Driver.IsTargetReached(s.Status))
		if !done && !completion.endpointDeadline.IsZero() && now.Sub(lastObserverReport) >= 250*time.Millisecond {
			logPDODiagnostic("[PDO-PP OBSERVE] drive=", d.Name, " goal=", completion.goal,
				" actual=", s.Position, " signedResidual=", completion.residual(s),
				" followingError=", s.FollowingError, " sw=", fmt.Sprintf("0x%04X", s.Status),
				" sample=", s.Sequence, " ", completion.diagnostic())
			lastObserverReport = now
		}
		r := completion.residual(s)
		if r < 0 {
			r = -r
		}
		if err != nil {
			return s.Position, r, err
		}
		if done {
			logPDODiagnostic("[PDO-PP ENDPOINT] drive=", d.Name, " goal=", completion.goal,
				" actual=", s.Position, " signedResidual=", completion.residual(s),
				" followingError=", s.FollowingError, " followingMapped=", s.FollowingErrorReady,
				" demand=", s.demand(completion.period), " sw=", fmt.Sprintf("0x%04X", s.Status),
				" sample=", s.Sequence, " sampleAt=", s.CapturedAt.Format(time.RFC3339Nano),
				" wkc=", s.WorkingCounter, " elapsed=", now.Sub(started))
			return s.Position, r, nil
		}
		time.Sleep(5 * time.Millisecond)
	}
}

// checkPositionEndpoint also covers movement during standby/clamping.
func checkPositionEndpoint(d *MasterDevice, goal int32) error {
	deadline := time.Now().Add(positionFeedbackMaxAge)
	var snapshot PDOFeedbackSnapshot
	for {
		if err := positionFeedbackStateError(d, true); err != nil {
			return err
		}
		s, ok := d.ReadPDOFeedback()
		age := time.Since(s.CapturedAt)
		if ok && s.Valid && age >= 0 && age <= positionCompletionMaxGap {
			if s.Status&0x0008 != 0 {
				return fmt.Errorf("[PDO-PP] drive fault during final check: sw=0x%04X err=0x%04X", s.Status, s.ErrorCode)
			}
			snapshot = s
			break
		}
		if !time.Now().Before(deadline) {
			return fmt.Errorf("[PDO-PP] no fresh valid feedback for final endpoint check")
		}
		time.Sleep(time.Millisecond)
	}
	actual := snapshot.Position
	residual := positionResidual(d, goal, actual)
	if residual < 0 {
		residual = -residual
	}
	if residual > int64(positionPulseTolerance) {
		return fmt.Errorf("%w: final check expected=%d actual=%d residualPulses=%d tolerancePulses=%d followingError=%d sw=0x%04X sample=%d",
			errPositionSettleTimeout, goal, actual, residual, positionPulseTolerance, snapshot.FollowingError, snapshot.Status, snapshot.Sequence)
	}
	return nil
}

// failPositionMove stops cyclic motion and latches this axis until Reset.
// Publish the alarm before the caller wakes the executor; the executor checks
// that cached alarm synchronously before executing any subsequent command.
func failPositionMove(d *MasterDevice, err error) error {
	d.posMoveAborted.Store(true)
	d.desiredTargetVelocity.Store(0)
	d.EnableJogPDO(false)
	d.EnablePosPDO(false)
	notifyDriverStatus("motor_running", "false", d)
	logger.Error("[PDO-PP] stopped drive=", d.Name, " error=", err)
	statusnotifier.Alarm(fmt.Sprintf("[PDO-PP] move failed: %v", err))
	return err
}

// freeRotate will not wait for ECS or will not send fin signal.
func freeRotate(masterDevice *MasterDevice, valueinDegree float64) error {
	logger.Trace("freeRotate to postion, driver", masterDevice.Name, "degree:", valueinDegree)
	err := doRotate(masterDevice, valueinDegree)
	if err != nil {
		return err
	}
	doneDriverAction()
	logger.Trace("freeRotate to postion completed, driver", masterDevice.Name)

	return nil
}

// doRotate function which power on and rotate to a degree, it waits for declamping and clamping.
func doRotate(masterDevice *MasterDevice, valueinDegree float64) error {
	delta := getPulsesFromDegree(masterDevice, helper.RoundFloatTo3(valueinDegree))
	actual := int64(masterDevice.PDOPos.Load())
	goal64 := actual + delta
	if masterDevice.Device.ContinuousRotation {
		target, err := makeMotionTarget(goal64, int32(actual), motionPeriod(masterDevice))
		if err != nil {
			return err
		}
		return runPositionGoal(masterDevice, target, "REL")
	}
	if goal64 < int64(-1<<31) || goal64 > int64(1<<31-1) {
		return fmt.Errorf("relative target overflow: actual=%d delta=%d goal=%d", actual, delta, goal64)
	}

	logger.Info("[PDO-PP REL] delta=", delta, " actual=", actual, " goal=", goal64)
	return runPositionGoal(masterDevice, MotionTarget{Position: int32(goal64)}, "REL")
}

// doRotateToAbsoluteGoal sends a precomputed, home-referenced encoder target.
// Unlike doRotate, it never reconstructs the target from the live position.
func doRotateToAbsoluteGoal(masterDevice *MasterDevice, goal MotionTarget) error {
	return runPositionGoal(masterDevice, goal, "ABS")
}

// runPositionGoal contains the common PDO motion, settle and clamp sequence.
// Both relative auxiliary moves and fixed absolute indexing targets use the
// same drive handshake and safety behavior.
func runPositionGoal(masterDevice *MasterDevice, target MotionTarget, targetKind string) error {
	goal := target.Position
	if masterDevice.Device.ContinuousRotation && (target.Period <= 0 || target.Period != motionPeriod(masterDevice)) {
		return fmt.Errorf("continuous motion target has an invalid period")
	}

	if !(IsPDOActive() && masterDevice.PdoPosReady) {
		return fmt.Errorf("PDO position mode not ready — SDO fallback removed")
	}

	if masterDevice.posMoveAborted.Load() || systemResetInProgress.Load() {
		return fmt.Errorf("position move blocked until Reset")
	}

	logger.Info("[PDO] Starting rotation in PP mode targetKind=", targetKind,
		" goal=", goal, " policy=CYCLIC_LATE_LATCH tolerancePulses=", positionPulseTolerance, " corrections=0")

	// Do not call FastPowerOn here. Once PDO cyclic control is active, the
	// CiA-402 cyclic task owns servo state. FastPowerOn calls pdoStopMotion(),
	// which disables the active PDO motion modes and can introduce a transient
	// standby cycle immediately before the Profile Position command.
	envSettings := settings.GetDriverSettings(masterDevice.Name)
	if _, declampErr := hasDeclamped(masterDevice, envSettings); declampErr != nil {
		logger.Error("[PDO-PP] doRotate: declamp failed, aborting move:", declampErr)
		statusnotifier.Alarm(declampErr.Error())
		return declampErr
	}

	notifyDriverStatus("motor_running", "true", masterDevice)

	// Store the new target before enabling PP output. Enabling first can let
	// the 1 ms cyclic task latch the previous target during this small window.
	var commandErr error
	if target.Period > 0 {
		command, err := target.relativeCommand(masterDevice.PDOPos.Load(), targetKind == "REL")
		if err != nil {
			notifyDriverStatus("motor_running", "false", masterDevice)
			return err
		}
		goal = command.Expected
		commandErr = masterDevice.SetRelativeMovePDO(command)
		logger.Info("[CONTINUOUS-PP] drive=", masterDevice.Name, " delta=", command.Value, " expected=", goal)
	} else {
		commandErr = masterDevice.SetTargetPositionPDO(goal)
	}
	if commandErr != nil {
		notifyDriverStatus("motor_running", "false", masterDevice)
		return commandErr
	}
	masterDevice.EnablePosPDO(true)

	moveStarted := time.Now()
	settledPosition, settledResidual, settleErr := waitForPositionCompletion(masterDevice)
	if settleErr != nil {
		return failPositionMove(masterDevice, settleErr)
	}
	// G91's endpoint is finalized by the acknowledged cyclic latch; G90's
	// fixed endpoint is never replaced with actual position.
	goal = masterDevice.desiredTargetPosition.Load()

	logger.Info("[PDO-PP SETTLED] drive=", masterDevice.Name,
		" targetKind=", targetKind,
		" goal=", goal,
		" actual=", settledPosition,
		" residualPulses=", settledResidual,
		" completionTime=", time.Since(moveStarted))

	// Keep PP active until the existing clamp sequence takes ownership.
	// That sequence may intentionally disable the servo after commanding the
	// brake. Its power/IO ordering remains unchanged; no new bit4 is generated.

	// FIN is allowed only after the clamp sequence has completed. Do not hide
	// clamp errors: an open/unknown clamp state must fail the movement.
	if _, clampErr := hasClamped(masterDevice, envSettings); clampErr != nil {
		return failPositionMove(masterDevice, fmt.Errorf("clamp failed: %w", clampErr))
	}
	if err := positionFeedbackStateError(masterDevice, envSettings.ClampDeclamp != 0); err != nil {
		return failPositionMove(masterDevice, err)
	}

	// The mechanical clamp can shift feedback after settling.
	// Validate the final endpoint again before the caller is allowed to FIN.
	if err := checkPositionEndpoint(masterDevice, goal); err != nil {
		return failPositionMove(masterDevice, err)
	}

	masterDevice.EnablePosPDO(false)
	notifyDriverStatus("motor_running", "false", masterDevice)
	logger.Info("[POST-REACH] phase=PP_DISABLED drive=", masterDevice.Name, " goal=", goal, " actual=", masterDevice.PDOPos.Load())

	logger.Info("[PDO-PP] rotation completed successfully targetKind=", targetKind,
		" goal=", goal, " final=", masterDevice.PDOPos.Load())

	return nil
}

// correctionForEndpoint builds only a short move to the SAME wrapped endpoint.
// It never replaces that endpoint with actual+the original program distance.
func correctionForEndpoint(goal, actual int32, period, maxPulses int64) (PPCommand, error) {
	if period < 2 || period > 1<<31-1 ||
		goal < 0 || int64(goal) >= period || actual < 0 || int64(actual) >= period {
		return PPCommand{}, fmt.Errorf("invalid correction coordinate: goal=%d actual=%d period=%d", goal, actual, period)
	}
	delta := rotaryDifference(int64(goal), int64(actual), period)
	magnitude := delta
	if magnitude < 0 {
		magnitude = -magnitude
	}
	if maxPulses > positionCorrectionMaxPulses {
		maxPulses = positionCorrectionMaxPulses
	}
	if maxPulses <= int64(positionPulseTolerance) ||
		magnitude <= int64(positionPulseTolerance) || magnitude > maxPulses {
		return PPCommand{}, fmt.Errorf("correction distance %d outside allowed range %d..%d pulses",
			delta, positionPulseTolerance+1, maxPulses)
	}
	return PPCommand{Value: int32(delta), Expected: goal, Reference: actual, LateBound: true}, nil
}

// correctPositionEndpoint runs only after the original move's bounded settling
// wait timed out. It issues a new relative set-point toward the original fixed
// endpoint, with all normal calibration checks still performed by the setter.
// It does not reset the drive, change parameters, or clear any abort flag.
func correctPositionEndpoint(d *MasterDevice, goal int32, attempt int) error {
	if !d.Device.ContinuousRotation || d.Device.DriveType != "a6_minas" {
		return fmt.Errorf("endpoint correction is only enabled for validated Panasonic continuous axes")
	}
	permitted := func() bool {
		healthy, _ := PDOHealthy()
		return IsPDOActive() && healthy && d.PdoPosReady &&
			!d.posMoveAborted.Load() && !systemResetInProgress.Load() &&
			!d.IsJogEnabled() && !d.pdoShutdownActive.Load() &&
			!d.pdoFaultResetActive.Load() && !d.pdoMTResetActive.Load() &&
			!d.pdoForcedServoCycleActive.Load() &&
			uint16(d.PDOStatus.Load()&0x006F) == 0x0027
	}
	if !permitted() || !d.IsPosEnabled() || d.ppSetpointPending.Load() {
		return fmt.Errorf("position correction cancelled or drive not ready")
	}

	actual := d.PDOPos.Load()
	// Bound the correction by both 100 pulses and 0.005 table degrees.
	// Integer division rounds down, so low-resolution axes cannot turn a
	// small pulse limit into a large angular movement.
	maxPulses := int64(d.Device.DriveXRatio) / 200
	command, err := correctionForEndpoint(goal, actual, motionPeriod(d), maxPulses)
	if err != nil {
		return err
	}
	direction := 1
	if command.Value < 0 {
		direction = -1
	}
	if isMotionBlockedByPotNotLimit(getCurrentDriverStatus(d.Device.Name), direction) {
		return fmt.Errorf("position correction blocked by POT/NOT limit")
	}

	// The existing setter refuses replacing an enabled PP command. End that
	// command, publish the correction through the same checked setter, then
	// re-enable PP. No dwell, clamp, or servo power cycle is inserted here.
	d.EnablePosPDO(false)
	if err := d.SetRelativeMovePDO(command); err != nil {
		return err
	}
	if !permitted() {
		return fmt.Errorf("position correction cancelled while preparing set-point")
	}
	d.EnablePosPDO(true)
	logger.Info("[PDO-PP CORRECTION] drive=", d.Name,
		" attempt=", attempt,
		" actual=", actual,
		" fixedGoal=", goal,
		" delta=", command.Value)

	// The next settling pass waits for ppSetpointPending=false and stable
	// drive completion within tolerance. A tiny correction may finish before
	// software observes bit10 LOW, so the large-move start detector is not used.
	return nil
}
