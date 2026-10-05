package acsolver

import (
	"fmt"
	"math"
	"time"
)

// Config holds solver parameters. Zero values are replaced by DefaultConfig
// defaults in NewWorld; changing parameters never invalidates converged state
// because XPBD solves positions from constraints every step.
type Config struct {
	// TimeStep is the fixed integration step of the reference clock.
	TimeStep time.Duration
	// Iterations is the number of XPBD Gauss-Seidel sweeps per step.
	Iterations int
	// Relaxation is the SOR multiplier applied to every positional
	// correction; valid range (0, 2]. 1 is standard XPBD.
	Relaxation float64
	// Compliance is the default inverse stiffness (m/N), used when a joint
	// does not specify its own. 0 means rigid.
	Compliance float64
	// LinearSleepSpeed: bodies whose linear speed stays below this for
	// SleepDelay enter sleep.
	LinearSleepSpeed float64
	// AngularSleepSpeed: angular analogue (rad/s).
	AngularSleepSpeed float64
	// SleepDelay is the required low-speed duration before sleeping.
	SleepDelay time.Duration
	// LinearDamping / AngularDamping are per-second velocity decay factors
	// in [0, 1]; the applied multiplier is pow(1-d, dt).
	LinearDamping  float64
	AngularDamping float64
	// MaxLinearSpeed / MaxAngularSpeed clamp velocities to avoid explosive
	// impulses; 0 disables clamping.
	MaxLinearSpeed  float64
	MaxAngularSpeed float64
	// RandomSeed drives the deterministic RNG (currently used for tie
	// breaking only). The same seed + same input sequence always yields
	// identical outputs.
	RandomSeed uint64
	// Parallel enables concurrent island solving. Island traversal, merging
	// and result order stay deterministic, so output is identical to serial.
	Parallel bool
}

// DefaultConfig returns a stable, conservative default configuration:
// 60 Hz fixed step, 8 iterations, standard relaxation, rigid constraints,
// gentle damping, sleeping enabled.
func DefaultConfig() Config {
	return Config{
		TimeStep:          16666667 * time.Nanosecond, // ~1/60 s, exact rational
		Iterations:        8,
		Relaxation:        1.0,
		Compliance:        0.0,
		LinearSleepSpeed:  0.02,
		AngularSleepSpeed: 0.02,
		SleepDelay:        500 * time.Millisecond,
		LinearDamping:     0.05,
		AngularDamping:    0.05,
		MaxLinearSpeed:    100.0,
		MaxAngularSpeed:   100.0,
		RandomSeed:        1,
		Parallel:          true,
	}
}

func (c *Config) validate() error {
	if c.TimeStep <= 0 {
		return wrapInvalid("TimeStep must be positive")
	}
	if c.Iterations <= 0 {
		return wrapInvalid("Iterations must be positive")
	}
	if math.IsNaN(c.Relaxation) || c.Relaxation <= 0 || c.Relaxation > 2 {
		return wrapInvalid("Relaxation must be within (0, 2]")
	}
	if math.IsNaN(c.Compliance) || c.Compliance < 0 {
		return wrapInvalid("Compliance must be non-negative")
	}
	if math.IsNaN(c.LinearSleepSpeed) || c.LinearSleepSpeed < 0 ||
		math.IsNaN(c.AngularSleepSpeed) || c.AngularSleepSpeed < 0 {
		return wrapInvalid("sleep speed thresholds must be non-negative")
	}
	if c.SleepDelay < 0 {
		return wrapInvalid("SleepDelay must be non-negative")
	}
	if math.IsNaN(c.LinearDamping) || c.LinearDamping < 0 || c.LinearDamping > 1 ||
		math.IsNaN(c.AngularDamping) || c.AngularDamping < 0 || c.AngularDamping > 1 {
		return wrapInvalid("damping factors must be within [0, 1]")
	}
	if math.IsNaN(c.MaxLinearSpeed) || math.IsNaN(c.MaxAngularSpeed) {
		return wrapInvalid("speed clamps must be finite")
	}
	return nil
}

func (c Config) dtSeconds() float64 {
	return float64(c.TimeStep) / float64(time.Second)
}

func wrapInvalid(msg string) error {
	return fmt.Errorf("%w: %s", ErrInvalidParameter, msg)
}
