package acsolver

import "math"

// Quat is a unit quaternion {X, Y, Z} (imaginary) and W (real).
type Quat struct {
	X, Y, Z, W float64
}

// IdentityQuat returns the identity rotation.
func IdentityQuat() Quat { return Quat{W: 1} }

// QuatAxisAngle builds a unit quaternion for a rotation of angle radians
// around axis. A non-finite or zero axis yields the identity rotation.
func QuatAxisAngle(axis Vec3, angle float64) Quat {
	if !axis.IsFinite() || math.IsNaN(angle) || math.IsInf(angle, 0) {
		return IdentityQuat()
	}
	n := axis.Normalized()
	h := angle * 0.5
	s := math.Sin(h)
	c := math.Cos(h)
	return Quat{n.X * s, n.Y * s, n.Z * s, c}
}

// Mul returns the Hamilton product q * o.
func (q Quat) Mul(o Quat) Quat {
	return Quat{
		X: q.W*o.X + q.X*o.W + q.Y*o.Z - q.Z*o.Y,
		Y: q.W*o.Y - q.X*o.Z + q.Y*o.W + q.Z*o.X,
		Z: q.W*o.Z + q.X*o.Y - q.Y*o.X + q.Z*o.W,
		W: q.W*o.W - q.X*o.X - q.Y*o.Y - q.Z*o.Z,
	}
}

// Rotate rotates vector v by q.
func (q Quat) Rotate(v Vec3) Vec3 {
	// v' = q * v * q^-1, expanded.
	qv := Vec3{q.X, q.Y, q.Z}
	t := qv.Cross(v).Scale(2)
	return v.Add(t.Scale(q.W)).Add(qv.Cross(t))
}

// Inverse returns the conjugate for unit quaternions (true inverse otherwise).
func (q Quat) Inverse() Quat {
	l2 := q.X*q.X + q.Y*q.Y + q.Z*q.Z + q.W*q.W
	if l2 == 0 {
		return IdentityQuat()
	}
	return Quat{-q.X / l2, -q.Y / l2, -q.Z / l2, q.W / l2}
}

// Normalized returns the unit quaternion; degenerate input maps to identity.
func (q Quat) Normalized() Quat {
	l := q.Length()
	if l == 0 || math.IsNaN(l) || math.IsInf(l, 0) {
		return IdentityQuat()
	}
	return Quat{q.X / l, q.Y / l, q.Z / l, q.W / l}
}

func (q Quat) Length() float64 {
	return math.Sqrt(q.X*q.X + q.Y*q.Y + q.Z*q.Z + q.W*q.W)
}

func (q Quat) IsFinite() bool {
	return !math.IsNaN(q.X) && !math.IsInf(q.X, 0) &&
		!math.IsNaN(q.Y) && !math.IsInf(q.Y, 0) &&
		!math.IsNaN(q.Z) && !math.IsInf(q.Z, 0) &&
		!math.IsNaN(q.W) && !math.IsInf(q.W, 0)
}

// RelativeTwist returns the signed rotation angle of the relative rotation
// qA^-1 * qB around the given world-space axis. The result lies in [-pi, pi].
func RelativeTwist(qA, qB Quat, axis Vec3) float64 {
	rel := qA.Inverse().Mul(qB)
	// Decompose rel into a twist around axis: project the rotation onto axis.
	// Rotation angle theta satisfies rel = (sin(theta/2)*n, cos(theta/2)).
	d := Vec3{rel.X, rel.Y, rel.Z}.Dot(axis)
	angle := 2 * math.Atan2(d, rel.W)
	return angle
}
