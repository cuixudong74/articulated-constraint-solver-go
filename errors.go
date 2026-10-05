package solver

import "fmt"

// ErrorKind classifies solver failures so callers can distinguish causes.
type ErrorKind int

const (
	ErrKindInvalidParameter ErrorKind = iota + 1 // illegal configuration or NaN input
	ErrKindSingular                              // singular / degenerate configuration
	ErrKindConflicting                           // conflicting constraints did not converge
)

// Error is a typed solver failure. The world remains steppable after it.
type Error struct {
	Kind    ErrorKind
	Message string
}

func (e *Error) Error() string {
	var k string
	switch e.Kind {
	case ErrKindInvalidParameter:
		k = "invalid parameter"
	case ErrKindSingular:
		k = "singular configuration"
	case ErrKindConflicting:
		k = "conflicting constraints"
	default:
		k = "unknown"
	}
	return fmt.Sprintf("solver: %s: %s", k, e.Message)
}

func invalidParamf(format string, args ...any) *Error {
	return &Error{Kind: ErrKindInvalidParameter, Message: fmt.Sprintf(format, args...)}
}
