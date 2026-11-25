package motordriver

import (
	"errors"
	"fmt"
	"time"
)

const (
	positionPulseTolerance   = int32(25)
	positionSettleTime       = 500 * time.Millisecond
	positionFeedbackMaxAge   = 50 * time.Millisecond
	positionCompletionMaxGap = 20 * time.Millisecond
	positionMoveTimeout      = 30 * time.Second
	positionEndpointTimeout  = 2 * time.Second
	positionEndpointMaxTime  = 4 * time.Second
)

var errPositionSettleTimeout = errors.New("residual pulse alarm")

// Both completion paths require valid endpoint dwell inside the same
// fixed 25-pulse band. Missing feedback never contributes settling time.
type positionCompletion struct {
	goal                                            int32
	period                                          int64
	deadline, endpointDeadline, endpointHardDeadline time.Time
	lastSequence                                    uint64

	sawBit10Low                   bool
	requireMotion, motionObserved bool
	startPosition                 int32

	lastInvalid, lastViolation, lastValidSample time.Time
	bandDwell, driveDwell                       time.Duration

	previousInBand, previousDriveEnd, pauseInterval bool
	previousCompletionCandidate                   bool
	completionAttempts                            uint64

	actualQuiet, demandQuiet         positionStability
	lastGate                         string
	lastActualQuiet, lastDemandQuiet bool

	invalidEvents, violationEvents, longGaps uint64
	maxGap                                  time.Duration
}

func newPositionCompletion(
	goal int32,
	period int64,
	started time.Time,
) positionCompletion {
	return positionCompletion{
		goal:     goal,
		period:   period,
		deadline: started.Add(positionMoveTimeout),
		lastGate: "await_feedback",
	}
}

func (c *positionCompletion) resetSoftwareQuiet() {
	c.actualQuiet.reset()
	c.demandQuiet.reset()
	c.lastActualQuiet = false
	c.lastDemandQuiet = false
}

func (c *positionCompletion) resetQuiet() {
	c.resetSoftwareQuiet()
	c.bandDwell = 0
	c.driveDwell = 0
	c.previousInBand = false
	c.previousDriveEnd = false
}

func (c *positionCompletion) armMotion(reference int32, distance int64) {
	c.startPosition = reference
	c.requireMotion = distance > int64(positionPulseTolerance) ||
		distance < -int64(positionPulseTolerance)
}

func (c *positionCompletion) residual(s PDOFeedbackSnapshot) int64 {
	if c.period > 0 {
		return rotaryDifference(
			int64(c.goal),
			int64(s.Position),
			c.period,
		)
	}
	return int64(c.goal) - int64(s.Position)
}

func (c *positionCompletion) diagnostic() string {
	return fmt.Sprintf(
		"gate=%s bandMs=%d driveMs=%d actualQuiet=%t demandQuiet=%t actualSpread=%d demandSpread=%d invalidEvents=%d endpointViolations=%d longGaps=%d maxGapMs=%d motionObserved=%t completionAttempts=%d",
		c.lastGate,
		c.bandDwell.Milliseconds(),
		c.driveDwell.Milliseconds(),
		c.lastActualQuiet,
		c.lastDemandQuiet,
		c.actualQuiet.maximum-c.actualQuiet.minimum,
		c.demandQuiet.maximum-c.demandQuiet.minimum,
		c.invalidEvents,
		c.violationEvents,
		c.longGaps,
		c.maxGap.Milliseconds(),
		c.motionObserved,
		c.completionAttempts,
	)
}

func (c *positionCompletion) failure(
	s PDOFeedbackSnapshot,
	reason string,
) error {
	r := c.residual(s)
	if r < 0 {
		r = -r
	}

	if r > int64(positionPulseTolerance) {
		return fmt.Errorf(
			"%w: %s expected=%d actual=%d signedResidual=%d residualPulses=%d tolerancePulses=%d followingError=%d followingMapped=%t demand=%d sw=0x%04X sample=%d %s",
			errPositionSettleTimeout,
			reason,
			c.goal,
			s.Position,
			c.residual(s),
			r,
			positionPulseTolerance,
			s.FollowingError,
			s.FollowingErrorReady,
			s.demand(c.period),
			s.Status,
			s.Sequence,
			c.diagnostic(),
		)
	}

	return fmt.Errorf(
		"[PDO-PP] completion timeout: %s expected=%d actual=%d signedResidual=%d followingError=%d sw=0x%04X sample=%d %s",
		reason,
		c.goal,
		s.Position,
		c.residual(s),
		s.FollowingError,
		s.Status,
		s.Sequence,
		c.diagnostic(),
	)
}

// Called only after the real ACK HIGH/LOW cycle.
// Only distinct, valid feedback intervals contribute settling time.
func (c *positionCompletion) observe(
	s PDOFeedbackSnapshot,
	now time.Time,
	modeReady, driveReached bool,
) (bool, error) {
	if s.Valid && s.Status&0x0008 != 0 {
		c.lastGate = "drive_fault"
		return false, fmt.Errorf(
			"[PDO-PP] drive fault: sw=0x%04X err=0x%04X",
			s.Status,
			s.ErrorCode,
		)
	}

	if s.Valid && s.Status&0x006F != 0x0027 {
		c.lastGate = "operation_disabled"
		return false, fmt.Errorf(
			"[PDO-PP] drive left Operation Enabled, sw=0x%04X",
			s.Status,
		)
	}

	if s.LastValidAt.IsZero() ||
		now.Sub(s.LastValidAt) > positionFeedbackMaxAge ||
		now.Before(s.LastValidAt) {
		c.lastGate = "stale_feedback"
		return false, fmt.Errorf(
			"[PDO-PP] stale or unavailable valid PDO feedback; %s",
			c.diagnostic(),
		)
	}

	if !now.Before(c.deadline) {
		return false, c.failure(s, "move deadline exceeded")
	}

	// This hard limit never restarts, even if completion is withdrawn.
	if !c.endpointHardDeadline.IsZero() &&
		!now.Before(c.endpointHardDeadline) {
		return false, c.failure(
			s,
			"endpoint hard settling deadline exceeded",
		)
	}

	if s.Sequence == c.lastSequence {
		return false, nil
	}
	c.lastSequence = s.Sequence

	if !s.Valid {
		c.pauseInterval = true
		c.resetSoftwareQuiet()
		c.lastGate = "incomplete_exchange"
		return false, nil
	}

	elapsed := time.Duration(0)

	if !c.lastValidSample.IsZero() {
		elapsed = s.CapturedAt.Sub(c.lastValidSample)

		if elapsed > c.maxGap {
			c.maxGap = elapsed
		}

		if elapsed <= 0 || elapsed > positionCompletionMaxGap {
			c.resetQuiet()
			c.longGaps++
			elapsed = 0
		}
	}
	c.lastValidSample = s.CapturedAt

	if !s.LastInvalidAt.Equal(c.lastInvalid) {
		c.lastInvalid = s.LastInvalidAt
		c.invalidEvents++
		c.pauseInterval = true
		c.resetSoftwareQuiet()
	}

	violation := !s.LastEndpointViolationAt.Equal(c.lastViolation)
	if violation {
		c.lastViolation = s.LastEndpointViolationAt
		c.violationEvents++
		c.bandDwell = 0
		c.previousInBand = false
	}

	// Unknown intervals contribute zero settling time.
	if c.pauseInterval {
		elapsed = 0
		c.pauseInterval = false
	}

	if now.Sub(s.CapturedAt) > positionCompletionMaxGap ||
		now.Before(s.CapturedAt) ||
		!modeReady ||
		s.Status&0x1000 != 0 {
		c.resetQuiet()
		c.lastGate = "mode_ack_or_sample_age"
		return false, nil
	}

	if s.Status&(1<<10) == 0 {
		c.sawBit10Low = true
	}

	if c.requireMotion && !c.motionObserved {
		delta := int64(s.Position) - int64(c.startPosition)

		if c.period > 0 {
			delta = rotaryDifference(
				int64(s.Position),
				int64(c.startPosition),
				c.period,
			)
		}

		c.motionObserved = delta > int64(positionPulseTolerance) ||
			delta < -int64(positionPulseTolerance)

		if !c.motionObserved {
			c.resetQuiet()
			c.lastGate = "await_motion"
			return false, nil
		}
	}

	c.lastActualQuiet = c.actualQuiet.observe(
		s.Position,
		c.period,
		s.CapturedAt,
		positionSettleTime,
		positionCompletionMaxGap,
	)

	c.lastDemandQuiet = false
	if s.FollowingErrorReady {
		c.lastDemandQuiet = c.demandQuiet.observe(
			s.demand(c.period),
			c.period,
			s.CapturedAt,
			positionSettleTime,
			positionCompletionMaxGap,
		)
	}

	r := c.residual(s)
	if r < 0 {
		r = -r
	}

	inBand := r <= int64(positionPulseTolerance)

	if inBand {
		if c.previousInBand && !violation {
			c.bandDwell += elapsed
		}
	} else {
		c.bandDwell = 0
	}
	c.previousInBand = inBand

	driveEnd := driveReached &&
		(c.sawBit10Low || (!c.requireMotion && inBand))

	if driveEnd {
		if c.previousDriveEnd {
			c.driveDwell += elapsed
		}
	} else {
		c.driveDwell = 0
	}
	c.previousDriveEnd = driveEnd

	softwareEnd := s.FollowingErrorReady &&
		c.lastActualQuiet &&
		c.lastDemandQuiet

	// A valid withdrawal followed by renewed completion earns a fresh
	// two-second budget. Missing feedback alone cannot restart it.
	// The fixed endpoint remains unchanged.
	completionCandidate := driveEnd || softwareEnd

	if completionCandidate && !c.previousCompletionCandidate {
		c.endpointDeadline = now.Add(positionEndpointTimeout)
		c.completionAttempts++

		if c.endpointHardDeadline.IsZero() {
			c.endpointHardDeadline = now.Add(positionEndpointMaxTime)
		}
	}
	c.previousCompletionCandidate = completionCandidate

	bandReady := c.bandDwell >= positionSettleTime
	driveReady := driveEnd && c.driveDwell >= positionSettleTime

	c.lastGate = "endpoint_band_dwell"

	if !inBand {
		c.lastGate = "residual_outside_25"
	} else if bandReady {
		c.lastGate = "drive_completion_or_software_quiet"

		if driveEnd && !driveReady {
			c.lastGate = "drive_dwell"
		}

		if !driveEnd && !c.lastActualQuiet {
			c.lastGate = "software_position_quiet"
		}

		if !driveEnd && c.lastActualQuiet && !c.lastDemandQuiet {
			c.lastGate = "software_demand_quiet"
		}
	}

	if inBand && bandReady && (driveReady || softwareEnd) {
		c.lastGate = "accepted"
		return true, nil
	}

	if completionCandidate &&
		!c.endpointDeadline.IsZero() &&
		!now.Before(c.endpointDeadline) {
		return false, c.failure(
			s,
			"endpoint did not settle within acceptance window",
		)
	}

	return false, nil
}