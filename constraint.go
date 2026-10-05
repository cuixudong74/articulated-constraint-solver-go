package acsolver

import "math"

// constraint is one scalar XPBD row over one or two dynamic bodies.
// Linear gradients ga/gb produce positional corrections (with lever-induced
// rotations); pure angular rows aa/ab produce orientation-only corrections.
type constraint struct {
	ia, ib     int // dynamic body indices; -1 when that end is static
	jointID    int
	limitRow   bool
	compliance float64

	la, lb Vec3 // anchor points in each body local frame
	ra, rb Vec3 // anchor levers (world space, refreshed on application)
	ga, gb Vec3 // linear gradients
	aa, ab Vec3 // angular correction axes
	c      float64
	w      float64
}

const singularEps = 1e-14

type constraintBuilder struct{}

func newConstraintBuilder() *constraintBuilder { return &constraintBuilder{} }

func worldAnchor(b *Body, l Vec3) Vec3 {
	return b.def.Position.Add(b.def.Rotation.Rotate(l))
}

// perpBasis returns two orthonormal vectors perpendicular to unit axis n.
func perpBasis(n Vec3) (u, v Vec3) {
	var ref Vec3
	if math.Abs(n.X) < 0.9 {
		ref = Vec3{X: 1}
	} else {
		ref = Vec3{Y: 1}
	}
	u = ref.Sub(n.Scale(ref.Dot(n))).Normalized()
	v = n.Cross(u).Normalized()
	return u, v
}

// angularResidual returns the world-space small rotation axis*angle that
// rotates body B relative to A away from their rest relative orientation.
func angularResidual(a, b *Body, restRel Quat) Vec3 {
	curRel := a.def.Rotation.Inverse().Mul(b.def.Rotation)
	dq := curRel.Mul(restRel.Inverse())
	if dq.W < 0 {
		dq = Quat{-dq.X, -dq.Y, -dq.Z, -dq.W}
	}
	xyz := Vec3{dq.X, dq.Y, dq.Z}
	angle := 2 * math.Atan2(xyz.Length(), dq.W)
	if angle > math.Pi {
		angle -= 2 * math.Pi
	}
	if xyz.Length() <= singularEps {
		return Vec3{}
	}
	return a.def.Rotation.Rotate(xyz.Normalized().Scale(angle))
}

// hingeSign returns the signed angle of body B's rotation about the hinge
// axis relative to the creation-time relative pose. The residual relative
// rotation delta = rel * restRel^-1 is expressed in body-A-local space and
// projected onto the hinge axis; atan2 over a perpendicular pin gives a
// continuous signed angle in [-pi, pi] without accumulated drift.
func hingeSign(j *Joint, a, b *Body) float64 {
	axisLocal := j.def.Axis
	var ref Vec3
	if math.Abs(axisLocal.X) < 0.9 {
		ref = Vec3{X: 1}
	} else {
		ref = Vec3{Y: 1}
	}
	u := ref.Sub(axisLocal.Scale(ref.Dot(axisLocal))).Normalized()
	v := axisLocal.Cross(u).Normalized()

	curRel := a.def.Rotation.Inverse().Mul(b.def.Rotation)
	dq := curRel.Mul(j.restRel.Inverse())
	if dq.W < 0 {
		dq = Quat{-dq.X, -dq.Y, -dq.Z, -dq.W}
	}
	// Rotate the body-A-local pin by the residual delta quaternion.
	pinRot := u
	rot := func(q Quat, vec Vec3) Vec3 {
		qv := Vec3{q.X, q.Y, q.Z}
		t := qv.Cross(vec).Scale(2)
		return vec.Add(t.Scale(q.W)).Add(qv.Cross(t))
	}
	dir := rot(dq, pinRot)
	return math.Atan2(dir.Dot(v), dir.Dot(u))
}

// build appends XPBD rows for joint j given the current pose.
func (cb *constraintBuilder) build(j *Joint, bodies []*Body, rows []constraint) []constraint {
	a, b := bodies[int(j.def.BodyA)], bodies[int(j.def.BodyB)]
	ia, ib := int(j.def.BodyA), int(j.def.BodyB)
	if a.def.Static {
		ia = -1
	}
	if b.def.Static {
		ib = -1
	}
	comp := j.def.Compliance

	linearRow := func(ga, gb Vec3, c float64, limit bool) constraint {
		return constraint{
			ia: ia, ib: ib, jointID: int(j.id), compliance: comp, limitRow: limit,
			la: j.def.AnchorA, lb: j.def.AnchorB,
			ga: ga, gb: gb, c: c,
		}
	}
	angularRow := func(aa, ab Vec3, c float64, limit bool) constraint {
		return constraint{
			ia: ia, ib: ib, jointID: int(j.id), compliance: comp, limitRow: limit,
			aa: aa, ab: ab, c: c,
		}
	}

	switch j.def.Kind {
	case JointDistance:
		wa := worldAnchor(a, j.def.AnchorA)
		wb := worldAnchor(b, j.def.AnchorB)
		d := wb.Sub(wa)
		l := d.Length()
		n := Vec3{}
		if l > singularEps {
			n = d.Scale(1 / l)
		}
		j.curDisplacement = l
		rows = append(rows, linearRow(n.Neg(), n, l-j.restDistance, false))

	case JointRevolute:
		wa := worldAnchor(a, j.def.AnchorA)
		wb := worldAnchor(b, j.def.AnchorB)
		d := wa.Sub(wb)
		axisA := a.def.Rotation.Rotate(j.def.Axis)
		axisB := b.def.Rotation.Rotate(j.def.Axis)
		n := axisA
		u, v := perpBasis(n)
		// Anchor coincidence: 3 linear rows.
		for k := 0; k < 3; k++ {
			var e Vec3
			switch k {
			case 0:
				e = Vec3{X: 1}
			case 1:
				e = Vec3{Y: 1}
			default:
				e = Vec3{Z: 1}
			}
			rows = append(rows, linearRow(e, e.Neg(), d.Dot(e), false))
		}
		// Lock the two non-hinge rotational DOFs.
		res := angularResidual(a, b, j.restRel)
		rows = append(rows, angularRow(u, u.Neg(), -res.Dot(u), false))
		rows = append(rows, angularRow(v, v.Neg(), -res.Dot(v), false))
		_ = axisB
		// Hinge angle limit.
		ang := hingeSign(j, a, b)
		j.curAngle = ang
		if j.def.HasAngleLimit {
			var viol float64
			if ang < j.def.MinAngle {
				viol = ang - j.def.MinAngle
			} else if ang > j.def.MaxAngle {
				viol = ang - j.def.MaxAngle
			}
			if viol != 0 {
				rows = append(rows, angularRow(n, n.Neg(), -viol, true))
			}
		}

	case JointPrismatic:
		axisA := a.def.Rotation.Rotate(j.def.Axis)
		u, v := perpBasis(axisA)
		wa := worldAnchor(a, j.def.AnchorA)
		wb := worldAnchor(b, j.def.AnchorB)
		rel := wb.Sub(wa)
		disp := rel.Dot(axisA) - j.baseDisplacement
		j.curDisplacement = disp
		// Block translation perpendicular to the slide axis.
		rows = append(rows, linearRow(u, u.Neg(), rel.Dot(u), false))
		rows = append(rows, linearRow(v, v.Neg(), rel.Dot(v), false))
		// Full rotation lock: residual decomposed on world basis.
		res := angularResidual(a, b, j.restRel)
		rows = append(rows, angularRow(u, u.Neg(), -res.Dot(u), false))
		rows = append(rows, angularRow(v, v.Neg(), -res.Dot(v), false))
		rows = append(rows, angularRow(axisA, axisA.Neg(), res.Dot(axisA), false))
		// Slide limits.
		if j.def.HasDispLimit {
			var viol float64
			if disp < j.def.MinDisplacement {
				viol = disp - j.def.MinDisplacement
			} else if disp > j.def.MaxDisplacement {
				viol = disp - j.def.MaxDisplacement
			}
			if viol != 0 {
				rows = append(rows, linearRow(axisA.Neg(), axisA, -viol, true))
			}
		}
	}
	return rows
}

// applyRow performs one XPBD / PBD-SOR correction against the *current*
// pose. Anchor levers are recomputed here (not cached) because earlier rows
// in the same Gauss-Seidel sweep may already have moved the body; caching
// them causes shared-body (star/chain) configurations to lock incorrectly.
func applyRow(r *constraint, bodies []*Body, dt2, relaxation float64) bool {
	var ba, bb *Body
	if r.ia >= 0 {
		ba = bodies[r.ia]
	}
	if r.ib >= 0 {
		bb = bodies[r.ib]
	}
	if ba != nil {
		r.ra = ba.def.Rotation.Rotate(r.la)
	}
	if bb != nil {
		r.rb = bb.def.Rotation.Rotate(r.lb)
	}

	w := 0.0
	if ba != nil {
		w += ba.invMass * r.ga.Length2()
		if t := r.ra.Cross(r.ga); t.Length2() > 0 {
			w += t.Dot(ba.worldInvI(t))
		}
		w += angularWeight(ba, r.aa)
	}
	if bb != nil {
		w += bb.invMass * r.gb.Length2()
		if t := r.rb.Cross(r.gb); t.Length2() > 0 {
			w += t.Dot(bb.worldInvI(t))
		}
		w += angularWeight(bb, r.ab)
	}
	r.w = w
	if w < singularEps {
		return false
	}
	alphaTilde := r.compliance / dt2
	dlambda := (-r.c / (w + alphaTilde)) * relaxation

	if ba != nil {
		if r.ga.Length2() > 0 {
			ba.def.Position = ba.def.Position.Add(r.ga.Scale(ba.invMass * dlambda))
			dw := ba.worldInvI(r.ra.Cross(r.ga)).Scale(dlambda)
			if dw.Length2() > 0 {
				ba.def.Rotation = QuatAxisAngle(dw, dw.Length()).Mul(ba.def.Rotation).Normalized()
			}
		}
		if r.aa.Length2() > 0 {
			dw := ba.worldInvI(r.aa).Scale(dlambda)
			ba.def.Rotation = QuatAxisAngle(dw, dw.Length()).Mul(ba.def.Rotation).Normalized()
		}
	}
	if bb != nil {
		if r.gb.Length2() > 0 {
			bb.def.Position = bb.def.Position.Add(r.gb.Scale(bb.invMass * dlambda))
			dw := bb.worldInvI(r.rb.Cross(r.gb)).Scale(dlambda)
			if dw.Length2() > 0 {
				bb.def.Rotation = QuatAxisAngle(dw, dw.Length()).Mul(bb.def.Rotation).Normalized()
			}
		}
		if r.ab.Length2() > 0 {
			dw := bb.worldInvI(r.ab).Scale(dlambda)
			bb.def.Rotation = QuatAxisAngle(dw, dw.Length()).Mul(bb.def.Rotation).Normalized()
		}
	}
	return true
}

func angularWeight(b *Body, axis Vec3) float64 {
	return axis.Dot(b.worldInvI(axis))
}

// ---- island integration --------------------------------------------------

type poseSnapshot struct {
	p  Vec3
	q  Quat
	lv Vec3
	av Vec3
}

func savePoses(bodies []*Body, members []int) []poseSnapshot {
	s := make([]poseSnapshot, len(members))
	for i, bi := range members {
		b := bodies[bi]
		s[i] = poseSnapshot{b.def.Position, b.def.Rotation, b.def.LinearVel, b.def.AngularVel}
	}
	return s
}

func restorePoses(bodies []*Body, members []int, s []poseSnapshot) {
	for i, bi := range members {
		b := bodies[bi]
		b.def.Position = s[i].p
		b.def.Rotation = s[i].q
		b.def.LinearVel = s[i].lv
		b.def.AngularVel = s[i].av
	}
}

func finiteBody(b *Body) bool {
	return b.def.Position.IsFinite() && b.def.Rotation.IsFinite() &&
		b.def.LinearVel.IsFinite() && b.def.AngularVel.IsFinite()
}

// solveIsland advances one island by dt seconds of its own clock using
// semi-implicit Euler + XPBD. It returns a failure (and rolls the island
// back to its pre-step pose) rather than letting non-finite values escape.
func solveIsland(is *island, dt float64, cfg Config, bodies []*Body, joints []*Joint) *SolveFailure {
	if is.allSleeping(bodies) {
		return nil
	}
	saved := savePoses(bodies, is.bodies)
	dt2 := dt * dt
	fail := stepIslandOnce(is, dt, dt2, cfg, bodies, joints)
	if fail != nil {
		restorePoses(bodies, is.bodies, saved)
		return fail
	}
	for _, bi := range is.bodies {
		if !finiteBody(bodies[bi]) {
			restorePoses(bodies, is.bodies, saved)
			return &SolveFailure{
				Kind: FailureNaNDetected, Island: is.id,
				BodyIDs: append([]int(nil), is.bodies...),
				Message: "non-finite pose/velocity after integration",
			}
		}
	}
	return nil
}

func stepIslandOnce(is *island, dt, dt2 float64, cfg Config, bodies []*Body, joints []*Joint) *SolveFailure {
	dampL := math.Pow(1-cfg.LinearDamping, dt)
	dampA := math.Pow(1-cfg.AngularDamping, dt)

	// 1) Semi-implicit velocity update from accumulated forces/impulses,
	//    then predict poses (xPred, qPred) by explicit integration.
	predP := make([]Vec3, len(is.bodies))
	predQ := make([]Quat, len(is.bodies))
	for i, bi := range is.bodies {
		b := bodies[bi]
		if b.sleeping {
			predP[i] = b.def.Position
			predQ[i] = b.def.Rotation
			continue
		}
		b.def.LinearVel = b.def.LinearVel.
			Add(b.force.Scale(b.invMass * dt)).
			Add(b.impulse.Scale(b.invMass))
		b.def.AngularVel = b.def.AngularVel.Add(b.worldInvI(b.torque).Scale(dt))
		b.def.LinearVel = clampVec(b.def.LinearVel, cfg.MaxLinearSpeed)
		b.def.AngularVel = clampVec(b.def.AngularVel, cfg.MaxAngularSpeed)
		predP[i] = b.def.Position.Add(b.def.LinearVel.Scale(dt))
		ang := b.def.AngularVel.Length()
		predQ[i] = QuatAxisAngle(b.def.AngularVel, ang*dt).Mul(b.def.Rotation).Normalized()
		// Start constraint solving from the predicted pose.
		b.def.Position = predP[i]
		b.def.Rotation = predQ[i]
	}

	// 2) XPBD Gauss-Seidel iterations in ascending joint order. Each joint's
	// rows are rebuilt from the *current* pose immediately before they are
	// applied, so corrections from earlier rows in the same sweep are seen by
	// later rows sharing a body (required for correct chain/star solving).
	builder := newConstraintBuilder()
	var singular *SolveFailure
	for it := 0; it < cfg.Iterations; it++ {
		for _, ji := range is.joints {
			j := joints[ji]
			if !j.enabled {
				continue
			}
			rows := builder.build(j, bodies, nil)
			for ri := range rows {
				if !applyRow(&rows[ri], bodies, dt2, cfg.Relaxation) {
					r := &rows[ri]
					if singular == nil && math.Abs(r.c) > 1e-3 {
						singular = &SolveFailure{
							Kind: FailureSingular, Island: is.id, JointID: r.jointID,
							Message: "constraint row has zero effective weight with non-zero residual",
						}
					}
				}
			}
		}
	}

	// 2b) Convergence check: rebuild rows against the solved pose. Any
	// non-limit row still carrying a large positional/angular residual
	// represents constraints that cannot be jointly satisfied (conflict).
	var conflict *SolveFailure
	{
		var check []constraint
		for _, ji := range is.joints {
			j := joints[ji]
			if !j.enabled {
				continue
			}
			check = builder.build(j, bodies, check)
		}
		for ri := range check {
			r := &check[ri]
			if r.limitRow {
				continue
			}
			if math.Abs(r.c) > conflictResidualTol {
				conflict = &SolveFailure{
					Kind: FailureConflictingConstraints, Island: is.id, JointID: r.jointID,
					Message: "constraints could not be jointly satisfied",
				}
				break
			}
		}
	}

	// 3) Derive velocities from the corrected pose, then damp and clear
	//    accumulators. Sleeping bodies keep zero velocity and frozen pose.
	for i, bi := range is.bodies {
		b := bodies[bi]
		if b.sleeping {
			continue
		}
		b.def.LinearVel = b.def.Position.Sub(predP[i]).Scale(1 / dt)
		// Angular velocity from finite rotation between predicted and solved
		// orientation: dq = qPred^-1 * qSolved, log -> world axis*angle / dt.
		dq := b.def.Rotation.Mul(predQ[i].Inverse())
		if dq.W < 0 {
			dq = Quat{-dq.X, -dq.Y, -dq.Z, -dq.W}
		}
		xyz := Vec3{dq.X, dq.Y, dq.Z}
		angle := 2 * math.Atan2(xyz.Length(), dq.W)
		if angle > math.Pi {
			angle -= 2 * math.Pi
		}
		if xyz.Length() > singularEps {
			b.def.AngularVel = xyz.Normalized().Scale(angle / dt)
		} else {
			b.def.AngularVel = Vec3{}
		}
		b.def.LinearVel = clampVec(b.def.LinearVel.Scale(dampL), cfg.MaxLinearSpeed)
		b.def.AngularVel = clampVec(b.def.AngularVel.Scale(dampA), cfg.MaxAngularSpeed)
		b.force = Vec3{}
		b.impulse = Vec3{}
		b.torque = Vec3{}
	}
	if singular != nil {
		return singular
	}
	return conflict
}

func clampVec(v Vec3, maxLen float64) Vec3 {
	if maxLen <= 0 {
		return v
	}
	if l := v.Length(); l > maxLen {
		return v.Scale(maxLen / l)
	}
	return v
}

// conflictResidualTol bounds the residual of a scalar row after all
// iterations. Residuals above it indicate contradictory constraints.
const conflictResidualTol = 0.05
