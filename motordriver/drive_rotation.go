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
	// The previous 500-count gate could itself exceed the complete indexing
	// scatter allowance (500 counts = 90 arcsec at 20000 counts/degree).
	// Ten counts is 1.8 arcsec at 20000 counts/degree and 0.3 arcsec at
	// 120000 counts/degree.
	positionPulseTolerance = int32(25)

	// Minimum hold after Target Reached. The settling helper then requires
	// stable completion within tolerance before PP is disabled.
	positionSettleTime = 100 * time.Millisecond
)

// Distinguish a settling deadline from faults, cancellation and lost feedback.
// Only a genuine settling timeout may enter endpoint correction.
var errPositionSettleTimeout = errors.New("position settle timeout")

const (
	positionCorrectionAttempts = 2
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

// hasTargetReached waits for CiA-402 PP completion: sw bit10=1 (Target Reached),
// stable for stableWindow. Uses statusword bits — not position error — because
// the drive runs its own ramp.
//
// HAL change: the per-drive completion condition (A6 requires bit12=0, Delta does
// not) is now delegated to masterDevice.Driver.IsTargetReached(sw). The old
// masterDevice.Device.DriveType string check has been removed.
func hasTargetReached(masterDevice *MasterDevice) error {
	// --- PDO path: Profile Position statusword check ---
	if masterDevice.PdoPosReady && IsPDOActive() {
		const (
			bitTargetReached = uint16(1 << 10) // SW bit10: motion complete, motor at target
			bitFault         = uint16(1 << 3)  // SW bit3:  drive fault
			moveTimeout      = 30 * time.Second

			// stableWindow: bit10 must stay HIGH continuously this long.
			// Time-based so single-sample noise glitches cause no latency penalty.
			stableWindow = 10 * time.Millisecond
		)

		logger.Trace("[PDO-PP] waiting for Target Reached. goal=",
			masterDevice.desiredTargetPosition.Load())

		// ----------------------------------------------------------
		// PHASE 1 — Wait for CiA-402 handshake completion
		// The cyclic task holds CW bit 4 HIGH until the drive replies
		// with SW bit 12 HIGH. Once acknowledged, ppSetpointPending
		// becomes false, and we can safely monitor for completion.
		// ----------------------------------------------------------
		handshakeDeadline := time.Now().Add(2 * time.Second)
		for masterDevice.ppSetpointPending.Load() {
			sw := uint16(masterDevice.PDOStatus.Load() & 0xFFFF)

			if sw&bitFault != 0 {
				return fmt.Errorf("[PDO-PP Phase1] drive fault during handshake: sw=0x%04X err=0x%04X",
					sw, uint16(masterDevice.PDOErr.Load()&0xFFFF))
			}
			if !IsPDOActive() {
				return fmt.Errorf("[PDO-PP Phase1] aborted: PDO cyclic stopped (system reset)")
			}
			if time.Now().After(handshakeDeadline) {
				return fmt.Errorf("[PDO-PP Phase1] handshake timeout: drive did not acknowledge set-point (bit 12)")
			}
			time.Sleep(1 * time.Millisecond)
		}

		logger.Debug("[PDO-PP] drive acknowledged set-point — waiting for bit10 to clear. goal=",
			masterDevice.desiredTargetPosition.Load(), " actual=", masterDevice.PDOPos.Load())

		// PHASE 1.5 — Wait for bit10 to go LOW (confirms new move started).
		//
		// CRITICAL: If bit10 never clears, it means the drive is still reporting
		// "Target Reached" from the PREVIOUS move. The motor physically never
		// started. We MUST NOT fall through to Phase 2 in this case — Phase 2
		// would immediately see bit10=1 (stale) and declare false "target reached",
		// causing FINISH BLOCKED and hanging the executor.
		//
		// If bit10 does not clear within the deadline, return a recoverable error
		// so the caller (doRotate/moveMotorToDegree) can retry the move.
		bit10Cleared := false
		bit10ClearDeadline := time.Now().Add(500 * time.Millisecond)
		for time.Now().Before(bit10ClearDeadline) {
			sw := uint16(masterDevice.PDOStatus.Load() & 0xFFFF)
			if sw&bitFault != 0 {
				return fmt.Errorf("[PDO-PP Phase1.5] drive fault: sw=0x%04X", sw)
			}
			if !IsPDOActive() {
				return fmt.Errorf("[PDO-PP Phase1.5] aborted: PDO cyclic stopped")
			}
			if sw&bitTargetReached == 0 {
				logger.Debug("[PDO-PP] bit10 cleared — motor moving...")
				bit10Cleared = true
				break
			}
			time.Sleep(1 * time.Millisecond)
		}

		// Motor did not start — drive ACKed the set-point but bit10 never went LOW.
		// This is a recoverable condition: the drive ignored the set-point (possibly
		// because it was in a transient "already at target" state). Return a specific
		// error so the caller can re-trigger the set-point and retry.
		if !bit10Cleared {
			goal := masterDevice.desiredTargetPosition.Load()
			actual := masterDevice.PDOPos.Load()
			residual := positionResidual(masterDevice, goal, actual)
			return fmt.Errorf("[PDO-PP Phase1.5] motor did not start — drive may have missed set-point. "+
				"goal=%d actual=%d residualPulses=%d. Retrying recommended.",
				goal, actual, residual)
		}

		// ----------------------------------------------------------
		// PHASE 2 — Wait for bit10 stable for stableWindow
		//
		// Use a time-based stability gate: track the wall-clock time
		// when bit10 first went HIGH. If it stays HIGH for stableWindow
		// without interruption, the move is complete.
		//
		// A glitch (bit10 drops to 0 for 1-2ms due to noise) resets
		// stableFrom but does NOT add fixed 50ms penalty — recovery is
		// instant on the next 1ms poll when bit10 returns to 1.
		// ----------------------------------------------------------
		timeout := time.Now().Add(moveTimeout)
		var stableFrom time.Time
		stableStarted := false

		for {
			sw := uint16(masterDevice.PDOStatus.Load() & 0xFFFF)

			// Abort immediately on drive fault (bit3)
			if sw&bitFault != 0 {
				return fmt.Errorf("[PDO-PP] drive fault during move: sw=0x%04X errCode=0x%04X",
					sw, uint16(masterDevice.PDOErr.Load()&0xFFFF))
			}

			// Exit immediately if PDO was shut down
			if !IsPDOActive() {
				return fmt.Errorf("[PDO-PP] aborted: PDO cyclic stopped (system reset)")
			}

			// Exit immediately if emergency cancelled this move.
			if masterDevice.posMoveAborted.Load() {
				masterDevice.posMoveAborted.Store(false)
				return fmt.Errorf("[PDO-PP] aborted: position move cancelled by emergency")
			}

			// HAL: IsTargetReached is drive-specific.
			//   A6Minas     → bit10=1 AND bit12=0 (strict Set-Point Ack handshake)
			//   DeltaASDA2E → bit10=1 only         (bit12 caused 30s hang on Delta)
			// Uses masterDevice.Driver (per-device) instead of GetMotorDriver() (global)
			// so that in a mixed-axis setup each axis uses its own completion condition.
			isTargetReached := masterDevice.Driver.IsTargetReached(sw)

			if isTargetReached {
				if !stableStarted {
					stableFrom = time.Now()
					stableStarted = true
				} else if time.Since(stableFrom) >= stableWindow {
					goalPulses := masterDevice.desiredTargetPosition.Load()
					actualPulses := masterDevice.PDOPos.Load()
					logger.Trace("[PDO-PP] target reached.",
						"sw=", fmt.Sprintf("0x%04X", sw),
						"goal=", goalPulses,
						"pos=", actualPulses,
						"residualPulses=", positionResidual(masterDevice, goalPulses, actualPulses))
					return nil
				}
			} else {
				stableStarted = false // Reset stability timer if it fluctuates
			}

			if time.Now().After(timeout) {
				goal := masterDevice.desiredTargetPosition.Load()
				actual := masterDevice.PDOPos.Load()
				return fmt.Errorf("[PDO-PP] timeout waiting for Target Reached: sw=0x%04X goal=%d pos=%d residualPulses=%d",
					sw, goal, actual, positionResidual(masterDevice, goal, actual))
			}
			time.Sleep(1 * time.Millisecond)
		}
	}

	return fmt.Errorf("hasTargetReached: PDO not active or PdoPosReady=false")
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

	logger.Info("[PDO] Starting rotation in PP mode targetKind=", targetKind,
		" goal=", goal)

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

	if err := hasTargetReached(masterDevice); err != nil {
		masterDevice.EnablePosPDO(false)
		// Clear the running flag so the user can press Reset.
		notifyDriverStatus("motor_running", "false", masterDevice)
		return err
	}

	settleStarted := time.Now()
	var settledPosition int32
	var settledResidual int64
	var settleErr error
	for correction := 0; ; correction++ {
		settledPosition, settledResidual, settleErr = waitForPositionSettle(masterDevice, goal)
		if settleErr == nil {
			break
		}

		// Recovery is limited to the validated Panasonic continuous path.
		// Never retry a drive fault, emergency/reset, stale PDO or large error.
		canCorrect := errors.Is(settleErr, errPositionSettleTimeout) &&
			masterDevice.Device.ContinuousRotation &&
			masterDevice.Device.DriveType == "a6_minas" &&
			correction < positionCorrectionAttempts &&
			settledResidual > int64(positionPulseTolerance) &&
			settledResidual <= positionCorrectionMaxPulses
		if canCorrect {
			if err := correctPositionEndpoint(masterDevice, goal, correction+1); err == nil {
				continue
			} else {
				settleErr = fmt.Errorf("%w; endpoint correction refused: %v", settleErr, err)
			}
		}

		masterDevice.EnablePosPDO(false)
		notifyDriverStatus("motor_running", "false", masterDevice)
		logger.Error("[PDO-PP SETTLE] drive=", masterDevice.Name,
			" targetKind=", targetKind, " error=", settleErr)
		return settleErr
	}

	logger.Info("[PDO-PP SETTLED] drive=", masterDevice.Name,
		" targetKind=", targetKind,
		" goal=", goal,
		" actual=", settledPosition,
		" residualPulses=", settledResidual,
		" settleTime=", time.Since(settleStarted))

	// PP remains active throughout settling. Only hand over to standby after
	// both drive completion and endpoint tolerance have stayed stable.
	masterDevice.EnablePosPDO(false)
	logger.Info("[POST-REACH] phase=PP_DISABLED drive=", masterDevice.Name,
		" goal=", goal, " actual=", masterDevice.PDOPos.Load())

	notifyDriverStatus("motor_running", "false", masterDevice)

	// FIN is allowed only after the clamp sequence has completed. Do not hide
	// clamp errors: an open/unknown clamp state must fail the movement.
	if _, clampErr := hasClamped(masterDevice, envSettings); clampErr != nil {
		logger.Error("[BRAKE-SEQ] phase=CLAMP_FAILED drive=", masterDevice.Name,
			" error=", clampErr)
		statusnotifier.Alarm(clampErr.Error())
		return clampErr
	}

	logger.Info("[PDO-PP] rotation completed successfully targetKind=", targetKind,
		" goal=", goal, " final=", masterDevice.PDOPos.Load())

	return nil
}

// waitForPositionSettle observes feedback without issuing another movement.
// The initial hold is retained, but one out-of-window sample now resets the
// stability timer instead of immediately failing the move. The deadline bounds
// the entire wait, including the initial hold. positionResidual preserves the
// existing normal/continuous endpoint calculation.
func waitForPositionSettle(masterDevice *MasterDevice, goal int32) (int32, int64, error) {
	const (
		settleTimeout = time.Second
		stableWindow = 50 * time.Millisecond
		pollInterval = 5 * time.Millisecond
		maxSampleGap = 20 * time.Millisecond
	)

	started := time.Now()
	deadline := started.Add(settleTimeout)
	lastSample := started
	var stableFrom time.Time

	for {
		now := time.Now()
		actual := masterDevice.PDOPos.Load()
		residual := positionResidual(masterDevice, goal, actual)
		if residual < 0 {
			residual = -residual
		}

		if !IsPDOActive() || !masterDevice.PdoPosReady {
			return actual, residual, fmt.Errorf("[PDO-PP SETTLE] aborted: PDO position feedback unavailable")
		}
		if masterDevice.posMoveAborted.Load() || !masterDevice.IsPosEnabled() {
			return actual, residual, fmt.Errorf("[PDO-PP SETTLE] aborted: position move cancelled")
		}
		if healthy, age := PDOHealthy(); !healthy {
			return actual, residual, fmt.Errorf("[PDO-PP SETTLE] aborted: stale PDO heartbeat, age=%s", age)
		}
		sw := uint16(masterDevice.PDOStatus.Load() & 0xFFFF)
		if sw&0x0008 != 0 {
			return actual, residual, fmt.Errorf("[PDO-PP SETTLE] drive fault: sw=0x%04X err=0x%04X",
				sw, uint16(masterDevice.PDOErr.Load()&0xFFFF))
		}
		if sw&0x006F != 0x0027 {
			return actual, residual, fmt.Errorf("[PDO-PP SETTLE] aborted: drive left Operation Enabled, sw=0x%04X", sw)
		}
		if !now.Before(deadline) {
			return actual, residual, fmt.Errorf("%w: expected=%d actual=%d residualPulses=%d tolerancePulses=%d sw=0x%04X elapsed=%s",
				errPositionSettleTimeout, goal, actual, residual, positionPulseTolerance, sw, now.Sub(started))
		}

		// A scheduling gap is not evidence of continuous stability.
		if now.Sub(lastSample) > maxSampleGap {
			stableFrom = time.Time{}
		}
		lastSample = now

		if now.Sub(started) >= positionSettleTime &&
			!masterDevice.ppSetpointPending.Load() &&
			residual <= int64(positionPulseTolerance) {
			if stableFrom.IsZero() {
				stableFrom = now
			} else if now.Sub(stableFrom) >= stableWindow {
				return actual, residual, nil
			}
		} else {
			stableFrom = time.Time{}
		}
		time.Sleep(pollInterval)
	}
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
	return PPCommand{Value: int32(delta), Expected: goal}, nil
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
