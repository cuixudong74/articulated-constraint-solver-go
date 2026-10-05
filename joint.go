package acsolver

import "math"

// JointKind enumerates supported joint types.
type JointKind int

const (
	// JointRevolute is a hinge: anchor points coincide, the only rotational
	// DOF is around Axis; MinAngle/MaxAngle bound the hinge angle.
	JointRevolute JointKind = iota
	// JointPrismatic is a slider: anchor motion along Axis is the only
	// translation DOF and rotation is fully locked; Min/MaxDisplacement bound
	// travel.
	JointPrismatic
	// JointDistance keeps the anchors at a fixed Distance.
	JointDistance
)

// String returns a stable name.
func (k JointKind) String() string {
	switch k {
	case JointRevolute:
		return "revolute"
	case JointPrismatic:
		return "prismatic"
	case JointDistance:
		return "distance"
	default:
		return "unknown"
	}
}

// JointDef describes a joint connecting two bodies.
type JointDef struct {
	Kind  JointKind
	BodyA BodyID
	BodyB BodyID
	// AnchorA / AnchorB are attachment points in each body's local frame.
	AnchorA Vec3
	AnchorB Vec3
	// Axis is the hinge/slide axis in body A's local frame. Must be non-zero;
	// it is normalized on creation.
	Axis Vec3
	// Distance is the rest length for JointDistance. <= 0 means use the
	// initial world distance at creation.
	Distance float64
	// MinAngle / MaxAngle in radians for JointRevolute limits.
	MinAngle, MaxAngle float64
	// MinDisplacement / MaxDisplacement in meters for JointPrismatic limits.
	MinDisplacement, MaxDisplacement float64
	// HasAngleLimit / HasDispLimit enable the corresponding limit pair.
	HasAngleLimit bool
	HasDispLimit  bool
	// Compliance overrides Config.Compliance for this joint when > 0.
	Compliance float64
}

// JointID identifies a joint inside a World.
type JointID int

// Joint is a constraint between two bodies.
type Joint struct {
	id      JointID
	def     JointDef
	enabled bool

	// Creation-time rest quantities (deterministic references).
	restDistance     float64
	baseDisplacement float64
	restRel          Quat // qA^-1 * qB captured at creation
	pin              Vec3 // body-B-local vector perpendicular to hinge axis

	// Last evaluated joint observables, refreshed during solving.
	curAngle        float64
	curDisplacement float64
}

func (j *Joint) ID() JointID        { return j.id }
func (j *Joint) Kind() JointKind    { return j.def.Kind }
func (j *Joint) BodyA() BodyID      { return j.def.BodyA }
func (j *Joint) BodyB() BodyID      { return j.def.BodyB }
func (j *Joint) Enabled() bool      { return j.enabled }
func (j *Joint) SetEnabled(on bool) { j.enabled = on }

func newJoint(id JointID, def JointDef, a, b *Body) (*Joint, error) {
	if def.BodyA == def.BodyB {
		return nil, ErrSelfReferencingJoint
	}
	if !def.AnchorA.IsFinite() || !def.AnchorB.IsFinite() || !def.Axis.IsFinite() {
		return nil, wrapInvalid("joint anchors or axis contain non-finite values")
	}
	if math.IsNaN(def.Distance) || math.IsInf(def.Distance, 0) || def.Distance < 0 {
		return nil, wrapInvalid("joint distance must be finite and non-negative")
	}
	if def.Compliance < 0 || math.IsNaN(def.Compliance) || math.IsInf(def.Compliance, 0) {
		return nil, wrapInvalid("joint compliance must be finite and non-negative")
	}
	j := &Joint{id: id, def: def, enabled: true}

	switch def.Kind {
	case JointRevolute, JointPrismatic:
		if def.Axis.IsZero(1e-12) {
			return nil, wrapInvalid("joint axis must be non-zero")
		}
		j.def.Axis = def.Axis.Normalized()
	case JointDistance:
		// axis unused
	}

	if def.HasAngleLimit {
		if math.IsNaN(def.MinAngle) || math.IsNaN(def.MaxAngle) || def.MinAngle > def.MaxAngle {
			return nil, wrapInvalid("angle limit must satisfy MinAngle <= MaxAngle")
		}
	}
	if def.HasDispLimit {
		if math.IsNaN(def.MinDisplacement) || math.IsNaN(def.MaxDisplacement) ||
			def.MinDisplacement > def.MaxDisplacement {
			return nil, wrapInvalid("displacement limit must satisfy Min <= Max")
		}
	}

	// Initial world-space rest quantities.
	wa := a.def.Position.Add(a.def.Rotation.Rotate(def.AnchorA))
	wb := b.def.Position.Add(b.def.Rotation.Rotate(def.AnchorB))
	if def.Kind == JointDistance {
		if def.Distance > 0 {
			j.restDistance = def.Distance
		} else {
			j.restDistance = wb.Sub(wa).Length()
		}
	}
	if def.Kind == JointPrismatic {
		axisW := a.def.Rotation.Rotate(j.def.Axis)
		j.baseDisplacement = wb.Sub(wa).Dot(axisW)
	}
	if def.Kind == JointRevolute || def.Kind == JointPrismatic {
		j.restRel = a.def.Rotation.Inverse().Mul(b.def.Rotation)
	}
	if def.Kind == JointRevolute {
		// Pin in B's local frame, perpendicular to the hinge axis.
		axisLocal := j.def.Axis
		var ref Vec3
		if math.Abs(axisLocal.X) < 0.9 {
			ref = Vec3{X: 1}
		} else {
			ref = Vec3{Y: 1}
		}
		j.pin = ref.Sub(axisLocal.Scale(ref.Dot(axisLocal))).Normalized()
	}
	return j, nil
}

// JointState is the exportable runtime state of a joint.
type JointState struct {
	ID    JointID
	Kind  JointKind
	BodyA BodyID
	BodyB BodyID
	// Angle is the signed hinge angle in radians (revolute).
	Angle float64
	// Displacement is the signed slide offset in meters (prismatic), or the
	// current anchor distance for distance joints.
	Displacement float64
}
