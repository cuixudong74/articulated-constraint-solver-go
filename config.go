package solver

// Config controls solver behaviour.
type Config struct {
	// VelocityIterations is the number of sequential-impulse iterations.
	VelocityIterations int
	// PositionIterations is the number of position-correction passes.
	PositionIterations int
	// Relaxation (Baumgarte factor in [0,1]) scales position-error bias.
	Relaxation float64
	// LinearSlop is the tolerated position error (meters).
	LinearSlop float64
	// AngularSlop is the tolerated angular error (radians).
	AngularSlop float64
	// MaxLinearCorrection caps per-iteration position correction.
	MaxLinearCorrection float64
	// MaxAngularCorrection caps per-iteration angular correction.
	MaxAngularCorrection float64
	// SleepLinearThreshold: speed below which a body may fall asleep.
	SleepLinearThreshold float64
	// SleepAngularThreshold: angular speed below which a body may sleep.
	SleepAngularThreshold float64
	// SleepTime: seconds below thresholds before sleeping.
	SleepTime float64
	// EnableSleep toggles the sleeping system.
	EnableSleep bool
	// ParallelWorkers caps goroutines used for island solving (<=1 = serial).
	ParallelWorkers int
}

// DefaultConfig returns a stable, well-tested configuration.
func DefaultConfig() Config {
	return Config{
		VelocityIterations:    10,
		PositionIterations:    8,
		Relaxation:            0.5,
		LinearSlop:            0.005,
		AngularSlop:           0.01,
		MaxLinearCorrection:   0.5,
		MaxAngularCorrection:  0.5,
		SleepLinearThreshold:  0.05,
		SleepAngularThreshold: 0.05,
		SleepTime:             0.5,
		EnableSleep:           true,
		ParallelWorkers:       4,
	}
}

func (c *Config) normalize() {
	if c.VelocityIterations <= 0 {
		c.VelocityIterations = 1
	}
	if c.PositionIterations <= 0 {
		c.PositionIterations = 1
	}
	c.Relaxation = clamp(c.Relaxation, 0, 1)
}
