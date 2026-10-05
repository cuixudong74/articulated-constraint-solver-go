package acsolver

import "math"

// BodyID identifies a rigid body inside a World.
type BodyID int

// BodyDef describes a rigid body at creation time.
type BodyDef struct {
	// Position / Rotation are the initial world pose.
	Position Vec3
	Rotation Quat
	// Mass in kilograms; <= 0 together with Static == false is illegal.
	Mass float64
	// InertiaDiag are the principal moments of inertia in body space.
	// Zero components are treated as infinite rotational resistance.
	InertiaDiag Vec3
	// LinearVel / AngularVel are initial world-space velocities.
	LinearVel  Vec3
	AngularVel Vec3
	// Static bodies have infinite mass, never move and never sleep.
	Static bool
	// TimeScale multiplies this body's reference clock; bodies connected by
	// a joint must share an effective island TimeScale (validated at world
	// level). 0 means 1.
	TimeScale float64
	// User is an opaque user attachment, never touched by the solver.
	User any
}

// Body is a rigid body simulation state. Fields are exported within the
// package for tight solver loops; external callers use the methods.
type Body struct {
	id  BodyID
	def BodyDef

	// Cached inverse quantities.
	invMass float64
	invILoc Vec3 // diagonal inverse inertia in body principal frame

	// Runtime accumulators.
	force    Vec3 // accumulated world-space forces (cleared after step)
	impulse  Vec3 // accumulated world-space linear impulses (cleared after step)
	torque   Vec3 // accumulated world-space torques (cleared after step)
	sleeping bool
	sleepT   float64 // seconds spent below sleep thresholds
}

func (b *Body) ID() BodyID            { return b.id }
func (b *Body) Position() Vec3        { return b.def.Position }
func (b *Body) Rotation() Quat        { return b.def.Rotation }
func (b *Body) LinearVelocity() Vec3  { return b.def.LinearVel }
func (b *Body) AngularVelocity() Vec3 { return b.def.AngularVel }
func (b *Body) Mass() float64         { return b.def.Mass }
func (b *Body) IsStatic() bool        { return b.def.Static }
func (b *Body) Sleeping() bool        { return b.sleeping }
func (b *Body) TimeScale() float64    { return b.def.TimeScale }

// SetPosition teleports the body. It does not wake the body; callers that
// need the chain awake should call World.Wake.
func (b *Body) SetPosition(p Vec3) { b.def.Position = p }

// SetRotation sets the body orientation and renormalizes it.
func (b *Body) SetRotation(q Quat) { b.def.Rotation = q.Normalized() }

func (b *Body) SetLinearVelocity(v Vec3)  { b.def.LinearVel = v }
func (b *Body) SetAngularVelocity(v Vec3) { b.def.AngularVel = v }

// ApplyForce accumulates a force applied at the center of mass.
func (b *Body) ApplyForce(f Vec3) { b.force = b.force.Add(f) }

// ApplyImpulse accumulates an instantaneous linear impulse at the center of
// mass (velocity change is applied during the next Step).
func (b *Body) ApplyImpulse(j Vec3) { b.impulse = b.impulse.Add(j) }

// ApplyTorque accumulates a world-space torque.
func (b *Body) ApplyTorque(t Vec3) { b.torque = b.torque.Add(t) }

// Wake marks the body awake and resets its sleep timer.
func (b *Body) Wake() {
	if b.def.Static {
		return
	}
	b.sleeping = false
	b.sleepT = 0
}

// PutToSleep forces the body to sleep and zeroes its velocities.
func (b *Body) PutToSleep() {
	if b.def.Static {
		return
	}
	b.sleeping = true
	b.def.LinearVel = Vec3{}
	b.def.AngularVel = Vec3{}
	b.force = Vec3{}
	b.impulse = Vec3{}
}

func newBody(id BodyID, def BodyDef) (*Body, error) {
	if !def.Position.IsFinite() || !def.LinearVel.IsFinite() || !def.AngularVel.IsFinite() {
		return nil, fmtInvalid("body pose or velocity contains non-finite values")
	}
	if !def.Rotation.IsFinite() {
		return nil, fmtInvalid("body rotation contains non-finite values")
	}
	if def.Static {
		def.Mass = 0
	} else {
		if math.IsNaN(def.Mass) || math.IsInf(def.Mass, 0) || def.Mass <= 0 {
			return nil, fmtInvalid("dynamic body mass must be positive and finite")
		}
	}
	for _, m := range []float64{def.InertiaDiag.X, def.InertiaDiag.Y, def.InertiaDiag.Z} {
		if math.IsNaN(m) || math.IsInf(m, 0) || m < 0 {
			return nil, fmtInvalid("inertia moments must be finite and non-negative")
		}
	}
	if def.TimeScale < 0 || math.IsNaN(def.TimeScale) || math.IsInf(def.TimeScale, 0) {
		return nil, fmtInvalid("TimeScale must be a finite non-negative number")
	}
	if def.TimeScale == 0 {
		def.TimeScale = 1
	}
	if def.Rotation == (Quat{}) {
		def.Rotation = IdentityQuat()
	} else {
		def.Rotation = def.Rotation.Normalized()
	}
	b := &Body{id: id, def: def}
	if !def.Static {
		b.invMass = 1.0 / def.Mass
	}
	b.invILoc = Vec3{
		invInertia(def.InertiaDiag.X),
		invInertia(def.InertiaDiag.Y),
		invInertia(def.InertiaDiag.Z),
	}
	return b, nil
}

func invInertia(m float64) float64 {
	if m == 0 {
		return 0 // infinite rotational resistance
	}
	return 1.0 / m
}

// worldInvI applies the diagonal body-space inverse inertia to a world axis.
func (b *Body) worldInvI(axisWorld Vec3) Vec3 {
	local := b.def.Rotation.Inverse().Rotate(axisWorld)
	out := Vec3{local.X * b.invILoc.X, local.Y * b.invILoc.Y, local.Z * b.invILoc.Z}
	return b.def.Rotation.Rotate(out)
}

func fmtInvalid(msg string) error { return &invalidParam{msg} }

type invalidParam struct{ msg string }

func (e *invalidParam) Error() string        { return "acsolver: invalid parameter: " + e.msg }
func (e *invalidParam) Is(target error) bool { return target == ErrInvalidParameter }
