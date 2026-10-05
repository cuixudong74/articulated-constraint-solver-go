package solver

import "math"

// This file contains the per-joint sequential-impulse solver implementations.
// Velocity iterations enforce Cdot = 0 (or inequality bounds for limits);
// position iterations apply clamped pseudo-impulses scaled by Relaxation.

const singularEps = 1e-9

// Impulse application guards each component by its inverse mass/inertia so
// that immovable bodies are never written. Besides being cheaper, this keeps
// parallel island solving data-race free: static bodies may be referenced by
// joints in several islands, and disjoint writes are the guarantee we need.
func applyImpulse(a, b *Body, p Vec2, rA, rB Vec2) {
	if a.InvMass != 0 {
		a.Velocity = a.Velocity.Sub(p.Scale(a.InvMass))
	}
	if a.InvInertia != 0 {
		a.AngularVelocity -= a.InvInertia * rA.Cross(p)
	}
	if b.InvMass != 0 {
		b.Velocity = b.Velocity.Add(p.Scale(b.InvMass))
	}
	if b.InvInertia != 0 {
		b.AngularVelocity += b.InvInertia * rB.Cross(p)
	}
}

func applyAngularImpulse(a, b *Body, lambda float64) {
	if a.InvInertia != 0 {
		a.AngularVelocity -= a.InvInertia * lambda
	}
	if b.InvInertia != 0 {
		b.AngularVelocity += b.InvInertia * lambda
	}
}

func applyPosImpulse(a, b *Body, p Vec2, rA, rB Vec2) {
	if a.InvMass != 0 {
		a.Position = a.Position.Sub(p.Scale(a.InvMass))
	}
	if a.InvInertia != 0 {
		a.Angle -= a.InvInertia * rA.Cross(p)
	}
	if b.InvMass != 0 {
		b.Position = b.Position.Add(p.Scale(b.InvMass))
	}
	if b.InvInertia != 0 {
		b.Angle += b.InvInertia * rB.Cross(p)
	}
}

func applyAngularPosImpulse(a, b *Body, lambda float64) {
	if a.InvInertia != 0 {
		a.Angle -= a.InvInertia * lambda
	}
	if b.InvInertia != 0 {
		b.Angle += b.InvInertia * lambda
	}
}

func worldAnchor(b *Body, local Vec2) (r Vec2, p Vec2) {
	r = local.Rotate(b.Angle)
	return r, b.Position.Add(r)
}

// ---------------------------------------------------------------- Revolute

func (j *Revolute) Type() JointType { return RevoluteJoint }

func (j *Revolute) initSolver(dt float64, cfg *Config) error {
	a, b := j.A, j.B
	j.rA = j.LocalAnchorA.Rotate(a.Angle)
	j.rB = j.LocalAnchorB.Rotate(b.Angle)

	// 2x2 effective mass for the point constraint.
	k00 := a.InvMass + b.InvMass + a.InvInertia*j.rA.Y*j.rA.Y + b.InvInertia*j.rB.Y*j.rB.Y
	k01 := -a.InvInertia*j.rA.X*j.rA.Y - b.InvInertia*j.rB.X*j.rB.Y
	k11 := a.InvMass + b.InvMass + a.InvInertia*j.rA.X*j.rA.X + b.InvInertia*j.rB.X*j.rB.X
	det := k00*k11 - k01*k01
	if math.Abs(det) < singularEps {
		return &Error{Kind: ErrKindSingular, Message: "revolute joint between two immovable bodies"}
	}
	invDet := 1 / det
	j.massLinear = [4]float64{k11 * invDet, -k01 * invDet, -k01 * invDet, k00 * invDet}

	// Angular limit.
	j.limitState = 0
	j.massLimit = 0
	if j.EnableLimit {
		if !(j.LowerAngle <= j.UpperAngle) {
			return invalidParamf("revolute limit lower %v > upper %v", j.LowerAngle, j.UpperAngle)
		}
		angle := b.Angle - a.Angle - j.ReferenceAngle
		ki := a.InvInertia + b.InvInertia
		if ki > singularEps {
			j.massLimit = 1 / ki
		}
		switch {
		case angle < j.LowerAngle:
			j.limitState = -1
		case angle > j.UpperAngle:
			j.limitState = 1
		}
	}
	return nil
}

func (j *Revolute) warmStart() {
	a, b := j.A, j.B
	applyImpulse(a, b, j.accLinear, j.rA, j.rB)
	if j.limitState != 0 {
		applyAngularImpulse(a, b, -float64(j.limitState)*j.accLimit)
	}
}

func (j *Revolute) solveVelocity() {
	a, b := j.A, j.B

	// Point constraint.
	cdot := b.Velocity.Add(crossSV(b.AngularVelocity, j.rB)).
		Sub(a.Velocity.Add(crossSV(a.AngularVelocity, j.rA)))
	m := j.massLinear
	p := Vec2{-(m[0]*cdot.X + m[1]*cdot.Y), -(m[2]*cdot.X + m[3]*cdot.Y)}
	j.accLinear = j.accLinear.Add(p)
	applyImpulse(a, b, p, j.rA, j.rB)

	// Angular limit (inequality, impulse clamped >= 0).
	if j.limitState != 0 && j.massLimit > 0 {
		s := -float64(j.limitState) // constraint g = s*(angle) >= 0
		cdotL := s * (b.AngularVelocity - a.AngularVelocity)
		lambda := -j.massLimit * cdotL
		newAcc := math.Max(0, j.accLimit+lambda)
		applyAngularImpulse(a, b, s*(newAcc-j.accLimit))
		j.accLimit = newAcc
	}
}

func (j *Revolute) solvePosition(cfg *Config) bool {
	a, b := j.A, j.B
	ok := true

	// Point constraint.
	rA, pA := worldAnchor(a, j.LocalAnchorA)
	rB, pB := worldAnchor(b, j.LocalAnchorB)
	c := pB.Sub(pA)
	if l := c.Len(); l > cfg.LinearSlop {
		ok = false
		k00 := a.InvMass + b.InvMass + a.InvInertia*rA.Y*rA.Y + b.InvInertia*rB.Y*rB.Y
		k01 := -a.InvInertia*rA.X*rA.Y - b.InvInertia*rB.X*rB.Y
		k11 := a.InvMass + b.InvMass + a.InvInertia*rA.X*rA.X + b.InvInertia*rB.X*rB.X
		det := k00*k11 - k01*k01
		if math.Abs(det) < singularEps {
			return false
		}
		invDet := 1 / det
		m := [4]float64{k11 * invDet, -k01 * invDet, -k01 * invDet, k00 * invDet}
		// Clamp correction magnitude, scale by relaxation.
		mag := clamp(l-cfg.LinearSlop, 0, cfg.MaxLinearCorrection) * cfg.Relaxation
		corr := c.Scale(mag / l)
		p := Vec2{-(m[0]*corr.X + m[1]*corr.Y), -(m[2]*corr.X + m[3]*corr.Y)}
		applyPosImpulse(a, b, p, rA, rB)
	}

	// Angular limit.
	if j.EnableLimit && j.limitState != 0 {
		angle := b.Angle - a.Angle - j.ReferenceAngle
		var errA float64
		if j.limitState < 0 {
			errA = angle - j.LowerAngle // must be >= 0
		} else {
			errA = j.UpperAngle - angle // must be >= 0
		}
		if errA < -cfg.AngularSlop {
			ok = false
			ki := a.InvInertia + b.InvInertia
			if ki > singularEps {
				s := -float64(j.limitState)
				corr := clamp(-errA-cfg.AngularSlop, 0, cfg.MaxAngularCorrection) * cfg.Relaxation
				applyAngularPosImpulse(a, b, s*corr/ki)
			}
		}
	}
	return ok
}

func (j *Revolute) resetAccumulators() {
	j.accLinear = Vec2{}
	j.accLimit = 0
}

func (j *Revolute) cloneState() jointState {
	return jointState{accLinear: j.accLinear, accLimit: j.accLimit}
}

func (j *Revolute) restoreState(s jointState) {
	j.accLinear = s.accLinear
	j.accLimit = s.accLimit
}

// ---------------------------------------------------------------- Prismatic

func (j *Prismatic) Type() JointType { return PrismaticJoint }

func (j *Prismatic) initSolver(dt float64, cfg *Config) error {
	a, b := j.A, j.B
	if j.LocalAxisA.LenSq() < singularEps {
		return invalidParamf("prismatic axis must be non-zero")
	}
	j.axis = j.LocalAxisA.Normalized().Rotate(a.Angle)
	j.perp = Vec2{-j.axis.Y, j.axis.X}
	j.rA = j.LocalAnchorA.Rotate(a.Angle)
	j.rB = j.LocalAnchorB.Rotate(b.Angle)

	denomP := a.InvMass + b.InvMass +
		a.InvInertia*sq(j.rA.Cross(j.perp)) + b.InvInertia*sq(j.rB.Cross(j.perp))
	if denomP < singularEps {
		return &Error{Kind: ErrKindSingular, Message: "prismatic joint between two immovable bodies"}
	}
	j.massPerp = 1 / denomP

	ki := a.InvInertia + b.InvInertia
	j.massAngular = 0
	if ki > singularEps {
		j.massAngular = 1 / ki
	}

	j.limitState = 0
	j.massLimit = 0
	if j.EnableLimit {
		if !(j.LowerTranslation <= j.UpperTranslation) {
			return invalidParamf("prismatic limit lower %v > upper %v", j.LowerTranslation, j.UpperTranslation)
		}
		d := b.Position.Add(j.rB).Sub(a.Position.Add(j.rA))
		t := d.Dot(j.axis)
		denomL := a.InvMass + b.InvMass +
			a.InvInertia*sq(j.rA.Cross(j.axis)) + b.InvInertia*sq(j.rB.Cross(j.axis))
		if denomL > singularEps {
			j.massLimit = 1 / denomL
		}
		switch {
		case t < j.LowerTranslation:
			j.limitState = -1
		case t > j.UpperTranslation:
			j.limitState = 1
		}
	}
	return nil
}

func (j *Prismatic) warmStart() {
	a, b := j.A, j.B
	applyImpulse(a, b, j.perp.Scale(j.accPerp), j.rA, j.rB)
	applyAngularImpulse(a, b, j.accAngular)
	if j.limitState != 0 {
		s := -float64(j.limitState)
		applyImpulse(a, b, j.axis.Scale(s*j.accLimit), j.rA, j.rB)
	}
}

func (j *Prismatic) solveVelocity() {
	a, b := j.A, j.B
	rel := b.Velocity.Add(crossSV(b.AngularVelocity, j.rB)).
		Sub(a.Velocity.Add(crossSV(a.AngularVelocity, j.rA)))

	// Perpendicular linear constraint.
	lambda := -j.massPerp * rel.Dot(j.perp)
	j.accPerp += lambda
	applyImpulse(a, b, j.perp.Scale(lambda), j.rA, j.rB)

	// Angular constraint.
	if j.massAngular > 0 {
		l := -j.massAngular * (b.AngularVelocity - a.AngularVelocity)
		j.accAngular += l
		applyAngularImpulse(a, b, l)
	}

	// Translation limit.
	if j.limitState != 0 && j.massLimit > 0 {
		s := -float64(j.limitState)
		cdot := s * rel.Dot(j.axis)
		l := -j.massLimit * cdot
		newAcc := math.Max(0, j.accLimit+l)
		applyImpulse(a, b, j.axis.Scale(s*(newAcc-j.accLimit)), j.rA, j.rB)
		j.accLimit = newAcc
	}
}

func (j *Prismatic) solvePosition(cfg *Config) bool {
	a, b := j.A, j.B
	ok := true
	axis := j.LocalAxisA.Normalized().Rotate(a.Angle)
	perp := Vec2{-axis.Y, axis.X}
	rA, pA := worldAnchor(a, j.LocalAnchorA)
	rB, pB := worldAnchor(b, j.LocalAnchorB)
	d := pB.Sub(pA)

	// Perpendicular error.
	if c := d.Dot(perp); math.Abs(c) > cfg.LinearSlop {
		ok = false
		denom := a.InvMass + b.InvMass +
			a.InvInertia*sq(rA.Cross(perp)) + b.InvInertia*sq(rB.Cross(perp))
		if denom > singularEps {
			mag := clamp(math.Abs(c)-cfg.LinearSlop, 0, cfg.MaxLinearCorrection) * cfg.Relaxation
			applyPosImpulse(a, b, perp.Scale(-math.Copysign(mag, c)/denom), rA, rB)
		}
	}

	// Angular error.
	if c := b.Angle - a.Angle - j.ReferenceAngle; math.Abs(c) > cfg.AngularSlop {
		ok = false
		ki := a.InvInertia + b.InvInertia
		if ki > singularEps {
			mag := clamp(math.Abs(c)-cfg.AngularSlop, 0, cfg.MaxAngularCorrection) * cfg.Relaxation
			applyAngularPosImpulse(a, b, -math.Copysign(mag, c)/ki)
		}
	}

	// Translation limit.
	if j.EnableLimit && j.limitState != 0 {
		t := d.Dot(axis)
		var errT float64
		if j.limitState < 0 {
			errT = t - j.LowerTranslation
		} else {
			errT = j.UpperTranslation - t
		}
		if errT < -cfg.LinearSlop {
			ok = false
			denom := a.InvMass + b.InvMass +
				a.InvInertia*sq(rA.Cross(axis)) + b.InvInertia*sq(rB.Cross(axis))
			if denom > singularEps {
				s := -float64(j.limitState)
				mag := clamp(-errT-cfg.LinearSlop, 0, cfg.MaxLinearCorrection) * cfg.Relaxation
				applyPosImpulse(a, b, axis.Scale(s*mag/denom), rA, rB)
			}
		}
	}
	return ok
}

func (j *Prismatic) resetAccumulators() {
	j.accPerp, j.accAngular, j.accLimit = 0, 0, 0
}

func (j *Prismatic) cloneState() jointState {
	return jointState{accLinear: Vec2{X: j.accPerp}, accAngular: j.accAngular, accLimit: j.accLimit}
}

func (j *Prismatic) restoreState(s jointState) {
	j.accPerp = s.accLinear.X
	j.accAngular = s.accAngular
	j.accLimit = s.accLimit
}

// ---------------------------------------------------------------- Distance

func (j *Distance) Type() JointType { return DistanceJoint }

func (j *Distance) initSolver(dt float64, cfg *Config) error {
	a, b := j.A, j.B
	if !(j.MinLength <= j.MaxLength) || j.MinLength < 0 {
		return invalidParamf("distance length range [%v, %v] invalid", j.MinLength, j.MaxLength)
	}
	j.rA = j.LocalAnchorA.Rotate(a.Angle)
	j.rB = j.LocalAnchorB.Rotate(b.Angle)
	d := b.Position.Add(j.rB).Sub(a.Position.Add(j.rA))
	l := d.Len()
	if l < 1e-9 {
		return &Error{Kind: ErrKindSingular, Message: "distance joint anchors coincide; axis undefined"}
	}
	j.u = d.Scale(1 / l)

	j.state = 0
	switch {
	case l > j.MaxLength:
		j.state = 1
	case l < j.MinLength:
		j.state = -1
	}

	denom := a.InvMass + b.InvMass +
		a.InvInertia*sq(j.rA.Cross(j.u)) + b.InvInertia*sq(j.rB.Cross(j.u))
	if denom < singularEps {
		return &Error{Kind: ErrKindSingular, Message: "distance joint between two immovable bodies"}
	}
	j.mass = 1 / denom
	return nil
}

func (j *Distance) warmStart() {
	if j.state == 0 {
		return
	}
	// state +1 pulls together (impulse -u on B), state -1 pushes apart.
	s := -float64(j.state)
	applyImpulse(j.A, j.B, j.u.Scale(s*j.acc), j.rA, j.rB)
}

func (j *Distance) solveVelocity() {
	if j.state == 0 {
		return
	}
	a, b := j.A, j.B
	rel := b.Velocity.Add(crossSV(b.AngularVelocity, j.rB)).
		Sub(a.Velocity.Add(crossSV(a.AngularVelocity, j.rA)))
	cdot := rel.Dot(j.u)
	s := -float64(j.state) // g = s*(L - limit) >= 0, gdot = s*cdot
	lambda := -j.mass * s * cdot
	newAcc := math.Max(0, j.acc+lambda)
	applyImpulse(a, b, j.u.Scale(s*(newAcc-j.acc)), j.rA, j.rB)
	j.acc = newAcc
}

func (j *Distance) solvePosition(cfg *Config) bool {
	if j.state == 0 {
		return true
	}
	a, b := j.A, j.B
	rA, pA := worldAnchor(a, j.LocalAnchorA)
	rB, pB := worldAnchor(b, j.LocalAnchorB)
	d := pB.Sub(pA)
	l := d.Len()
	if l < 1e-9 {
		return false
	}
	u := d.Scale(1 / l)
	var errL float64
	if j.state > 0 {
		errL = j.MaxLength - l // must be >= 0
	} else {
		errL = l - j.MinLength
	}
	if errL >= -cfg.LinearSlop {
		return true
	}
	denom := a.InvMass + b.InvMass +
		a.InvInertia*sq(rA.Cross(u)) + b.InvInertia*sq(rB.Cross(u))
	if denom < singularEps {
		return false
	}
	s := -float64(j.state)
	mag := clamp(-errL-cfg.LinearSlop, 0, cfg.MaxLinearCorrection) * cfg.Relaxation
	applyPosImpulse(a, b, u.Scale(s*mag/denom), rA, rB)
	return false
}

func (j *Distance) resetAccumulators() { j.acc = 0 }

func (j *Distance) cloneState() jointState {
	return jointState{accLimit: j.acc}
}

func (j *Distance) restoreState(s jointState) {
	j.acc = s.accLimit
}

func sq(x float64) float64 { return x * x }
