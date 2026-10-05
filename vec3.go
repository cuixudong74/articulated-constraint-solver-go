package acsolver

import "math"

// Vec3 is a 3D vector with float64 components.
type Vec3 struct {
	X, Y, Z float64
}

// Vec constructs a Vec3.
func Vec(x, y, z float64) Vec3 { return Vec3{X: x, Y: y, Z: z} }

func (v Vec3) Add(o Vec3) Vec3      { return Vec3{v.X + o.X, v.Y + o.Y, v.Z + o.Z} }
func (v Vec3) Sub(o Vec3) Vec3      { return Vec3{v.X - o.X, v.Y - o.Y, v.Z - o.Z} }
func (v Vec3) Neg() Vec3            { return Vec3{-v.X, -v.Y, -v.Z} }
func (v Vec3) Scale(s float64) Vec3 { return Vec3{v.X * s, v.Y * s, v.Z * s} }

func (v Vec3) Dot(o Vec3) float64 { return v.X*o.X + v.Y*o.Y + v.Z*o.Z }

func (v Vec3) Cross(o Vec3) Vec3 {
	return Vec3{
		v.Y*o.Z - v.Z*o.Y,
		v.Z*o.X - v.X*o.Z,
		v.X*o.Y - v.Y*o.X,
	}
}

func (v Vec3) Length2() float64 { return v.Dot(v) }

func (v Vec3) Length() float64 { return math.Sqrt(v.Length2()) }

// Normalized returns the unit vector. The zero vector maps to itself so that
// callers never observe NaN; use IsZero to guard singular axes explicitly.
func (v Vec3) Normalized() Vec3 {
	l := v.Length()
	if l == 0 || math.IsNaN(l) || math.IsInf(l, 0) {
		return Vec3{}
	}
	return v.Scale(1.0 / l)
}

func (v Vec3) IsFinite() bool {
	return !math.IsNaN(v.X) && !math.IsInf(v.X, 0) &&
		!math.IsNaN(v.Y) && !math.IsInf(v.Y, 0) &&
		!math.IsNaN(v.Z) && !math.IsInf(v.Z, 0)
}

func (v Vec3) IsZero(eps float64) bool {
	return math.Abs(v.X) <= eps && math.Abs(v.Y) <= eps && math.Abs(v.Z) <= eps
}
