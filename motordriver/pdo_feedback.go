package motordriver

import (
	"sync/atomic"
	"time"
)

// PDOFeedbackSnapshot contains one domain-process result. A sample with
// Valid=false must never advance a motion handshake or completion timer.
// LastValidAt is retained across incomplete working counters.
type PDOFeedbackSnapshot struct {
	Sequence                               uint64
	CapturedAt, LastValidAt                time.Time
	LastInvalidAt, LastEndpointViolationAt time.Time
	Valid                                  bool
	Position                               int32
	Status, ErrorCode                      uint16
	Mode                                   int8
	FollowingError                         int32 // signed 0x60F4, in drive command units
	FollowingErrorReady                    bool
	WorkingCounter                         uint32
	EndpointViolation                      bool // publication input for an active move
}

// A single cyclic writer publishes without allocating or taking a mutex.
// Every payload field is atomic, including during a failed reader attempt.
type pdoFeedbackCache struct {
	sequence                   atomic.Uint64
	captured, lastValid        atomic.Int64
	lastInvalid, lastViolation atomic.Int64
	position, mode, following  atomic.Int32
	status, errorCode, counter atomic.Uint32
	valid, followingReady      atomic.Bool
}

func (c *pdoFeedbackCache) publish(s PDOFeedbackSnapshot) {
	c.sequence.Add(1)
	c.captured.Store(s.CapturedAt.UnixNano())
	if s.Valid {
		c.lastValid.Store(s.CapturedAt.UnixNano())
		if s.EndpointViolation {
			c.lastViolation.Store(s.CapturedAt.UnixNano())
		}
	} else {
		c.lastInvalid.Store(s.CapturedAt.UnixNano())
	}
	c.position.Store(s.Position)
	c.status.Store(uint32(s.Status))
	c.errorCode.Store(uint32(s.ErrorCode))
	c.mode.Store(int32(s.Mode))
	c.following.Store(s.FollowingError)
	c.counter.Store(s.WorkingCounter)
	c.valid.Store(s.Valid)
	c.followingReady.Store(s.FollowingErrorReady)
	c.sequence.Add(1)
}

func (c *pdoFeedbackCache) load() (PDOFeedbackSnapshot, bool) {
	for attempt := 0; attempt < 8; attempt++ {
		before := c.sequence.Load()
		if before == 0 || before&1 != 0 {
			continue
		}
		s := PDOFeedbackSnapshot{
			Sequence: before, CapturedAt: time.Unix(0, c.captured.Load()),
			Position: c.position.Load(), Status: uint16(c.status.Load()),
			ErrorCode: uint16(c.errorCode.Load()), Mode: int8(c.mode.Load()),
			FollowingError: c.following.Load(), FollowingErrorReady: c.followingReady.Load(),
			WorkingCounter: c.counter.Load(), Valid: c.valid.Load(),
		}
		if last := c.lastValid.Load(); last != 0 {
			s.LastValidAt = time.Unix(0, last)
		}
		if last := c.lastInvalid.Load(); last != 0 {
			s.LastInvalidAt = time.Unix(0, last)
		}
		if last := c.lastViolation.Load(); last != 0 {
			s.LastEndpointViolationAt = time.Unix(0, last)
		}
		if c.sequence.Load() == before {
			return s, true
		}
	}
	return PDOFeedbackSnapshot{}, false
}

func (s PDOFeedbackSnapshot) demand(period int64) int32 {
	v := int64(s.Position) + int64(s.FollowingError)
	if period > 0 {
		v = wrapRotary(v, period)
	}
	return int32(v)
}
