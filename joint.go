package solver

// JointType identifies the joint kind.
type JointType int

const (
	RevoluteJoint  JointType = iota // pins two bodies at a shared anchor point
	PrismaticJoint                  // sliding along a fixed axis
	DistanceJoint                   // keeps anchor distance within a range
)

// Joint is a constraint between two bodies.
type Joint interface {
	// Type returns the joint kind.
	Type() JointType
	// Bodies returns the connected bodies (A, B).
	Bodies() (*Body, *Body)
	// initSolver precomputes step-invariant data; may report singular config.
	initSolver(dt float64, cfg *Config) error
	// warmStart applies accumulated impulses.
	warmStart()
	// solveVelocity runs one velocity iteration.
	solveVelocity()
	// solvePosition runs one position-correction pass; reports convergence.
	solvePosition(cfg *Config) bool
	// resetAccumulators clears accumulated impulses (used on restore).
	resetAccumulators()
	// cloneState / restoreState support snapshots.
	cloneState() jointState
	restoreState(s jointState)
}

// jointState holds per-joint solver state for snapshots.
type jointState struct {
	accLinear  Vec2
	accAngular float64
	accLimit   float64
}

// jointBase is embedded by concrete joints.
type jointBase struct {
	A, B *Body
}

func (j *jointBase) Bodies() (*Body, *Body) { return j.A, j.B }

// Revolute pins local anchors of A and B together, with optional angle limits.
type Revolute struct {
	jointBase
	LocalAnchorA, LocalAnchorB Vec2
	ReferenceAngle             float64
	EnableLimit                bool
	LowerAngle, UpperAngle     float64

	// solver state
	rA, rB      Vec2
	accLinear   Vec2
	accAngular  float64
	accLimit    float64
	limitState  int        // 0 inactive, -1 at lower, +1 at upper
	massLinear  [4]float64 // 2x2 effective mass
	massAngular float64
	massLimit   float64
}

// Prismatic constrains B to slide along an axis fixed on A, with optional
// translation limits.
type Prismatic struct {
	jointBase
	LocalAnchorA, LocalAnchorB         Vec2
	LocalAxisA                         Vec2 // unit axis in A's frame
	ReferenceAngle                     float64
	EnableLimit                        bool
	LowerTranslation, UpperTranslation float64

	// solver state
	axis, perp  Vec2
	rA, rB      Vec2
	accPerp     float64
	accAngular  float64
	accLimit    float64
	limitState  int
	massPerp    float64
	massAngular float64
	massLimit   float64
}

// Distance keeps the world anchors within [MinLength, MaxLength].
type Distance struct {
	jointBase
	LocalAnchorA, LocalAnchorB Vec2
	MinLength, MaxLength       float64

	// solver state
	u      Vec2 // world axis A->B
	rA, rB Vec2
	acc    float64
	mass   float64
	state  int // 0 at rest, -1 at min, +1 at max
}
