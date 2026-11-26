package motordriver

import (
	logger "EtherCAT/logger"
	"fmt"
	"sync/atomic"
	"time"
)

// Only informational diagnostics use this buffer. PDO feedback, commands,
// completion decisions, failure latches and alarms never pass through it.
// One writer owns output; a full buffer discards a diagnostic instead of
// waiting for a terminal, file, or the shared logger mutex.
const pdoDiagnosticCapacity = 256

type pdoDiagnosticRecord struct {
	capturedAt time.Time
	args       []interface{}
}

type pdoDiagnosticBuffer struct {
	records   chan pdoDiagnosticRecord
	discarded atomic.Uint64
}

func newPDODiagnosticBuffer(capacity int) *pdoDiagnosticBuffer {
	return &pdoDiagnosticBuffer{records: make(chan pdoDiagnosticRecord, capacity)}
}

func (b *pdoDiagnosticBuffer) enqueue(record pdoDiagnosticRecord) bool {
	select {
	case b.records <- record:
		return true
	default:
		b.discarded.Add(1)
		return false
	}
}

// The caller must supply immutable values, not pointers to live motion state.
// All call sites in this patch supply numbers, strings, booleans or durations.
// In particular, completion.diagnostic() is evaluated by its owning goroutine.
func (b *pdoDiagnosticBuffer) info(args ...interface{}) {
	b.enqueue(pdoDiagnosticRecord{capturedAt: time.Now(), args: args})
}

var pdoDiagnostics = newPDODiagnosticBuffer(pdoDiagnosticCapacity)

func init() {
	go func() {
		for record := range pdoDiagnostics.records {
			// Outer logger timestamps now describe writing time. eventAt preserves
			// the time this diagnostic was queued; sampleAt, where present, still
			// describes the actual feedback snapshot. Existing message tags stay.
			logger.Info("[PDO-DIAG] eventAt=", record.capturedAt.Format(time.RFC3339Nano),
				" diagnosticsDropped=", pdoDiagnostics.discarded.Load(), " ",
				fmt.Sprint(record.args...))
		}
	}()
}

func logPDODiagnostic(args ...interface{}) {
	pdoDiagnostics.info(args...)
}
