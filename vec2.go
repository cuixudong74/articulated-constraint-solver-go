package solver

import "math"

// Vec2 is a 2D vector.
type Vec2 struct {
	X, Y float64
}

func (a Vec2) Add(b Vec2) Vec2      { return Vec2{a.X + b.X, a.Y + b.Y} }
func (a Vec2) Sub(b Vec2) Vec2      { return Vec2{a.X - b.X, a.Y - b.Y} }
func (a Vec2) Scale(s float64) Vec2 { return Vec2{a.X * s, a.Y * s} }
func (a Vec2) Neg() Vec2            { return Vec2{-a.X, -a.Y} }
func (a Vec2) Dot(b Vec2) float64   { return a.X*b.X + a.Y*b.Y }
func (a Vec2) Cross(b Vec2) float64 { return a.X*b.Y - a.Y*b.X }
func (a Vec2) Len() float64         { return math.Hypot(a.X, a.Y) }
func (a Vec2) LenSq() float64       { return a.X*a.X + a.Y*a.Y }

func (a Vec2) Normalized() Vec2 {
	l := a.Len()
	if l < 1e-12 {
		return Vec2{}
	}
	return a.Scale(1 / l)
}

// Rotate rotates v by angle (radians).
func (a Vec2) Rotate(angle float64) Vec2 {
	c, s := math.Cos(angle), math.Sin(angle)
	return Vec2{c*a.X - s*a.Y, s*a.X + c*a.Y}
}

// crossSV returns s x v (scalar cross vector) = (-s*v.Y, s*v.X).
func crossSV(s float64, v Vec2) Vec2 { return Vec2{-s * v.Y, s * v.X} }

func clamp(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
