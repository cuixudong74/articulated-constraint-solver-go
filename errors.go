package acsolver

import (
	"errors"
	"fmt"
)

// Sentinel errors returned at construction/configuration time. Callers can
// distinguish failure reasons with errors.Is.
var (
	// ErrInvalidParameter covers NaN, negative mass/time step, zero axes, etc.
	ErrInvalidParameter = errors.New("acsolver: invalid parameter")
	// ErrUnknownBody is returned when a BodyID does not exist.
	ErrUnknownBody = errors.New("acsolver: unknown body")
	// ErrUnknownJoint is returned when a JointID does not exist.
	ErrUnknownJoint = errors.New("acsolver: unknown joint")
	// ErrConflictingJoint is returned when a joint duplicates another joint
	// between the same pair of bodies with an incompatible definition.
	ErrConflictingJoint = errors.New("acsolver: conflicting joint on bodies")
	// ErrSelfReferencingJoint is returned when both joint ends are one body.
	ErrSelfReferencingJoint = errors.New("acsolver: self referencing joint")
)

// FailureKind classifies a runtime solving failure recorded in StepReport.
type FailureKind int

const (
	// FailureNone means no failure.
	FailureNone FailureKind = iota
	// FailureSingular means a constraint row had a zero effective weight in a
	// degenerate configuration (e.g. colinear hinge axes).
	FailureSingular
	// FailureConflictingConstraints means incompatible constraints could not
	// be satisfied simultaneously (e.g. two distance joints with different
	// lengths on the same pair).
	FailureConflictingConstraints
	// FailureNaNDetected means a non-finite value appeared; the island step
	// was rolled back so the world remains finite and continueable.
	FailureNaNDetected
	// FailureInvalidParameter means configuration values were illegal.
	FailureInvalidParameter
)

// String returns a stable human-readable name.
func (k FailureKind) String() string {
	switch k {
	case FailureSingular:
		return "singular configuration"
	case FailureConflictingConstraints:
		return "conflicting constraints"
	case FailureNaNDetected:
		return "non-finite value detected"
	case FailureInvalidParameter:
		return "invalid parameter"
	default:
		return "none"
	}
}

// SolveFailure describes a per-island failure recorded during a step. It
// implements error so failures can be surfaced uniformly.
type SolveFailure struct {
	Kind    FailureKind
	Island  int
	BodyIDs []int
	JointID int
	Message string
}

func (f *SolveFailure) Error() string {
	return fmt.Sprintf("acsolver: island %d: %s (%s)", f.Island, f.Kind, f.Message)
}
