package solver

import (
	"math"
	"testing"
)

const dt = 1.0 / 60.0

// makeChain builds a hanging chain: static anchor at top, n links of length
// 1 connected by revolute joints, hanging straight down.
func makeChain(cfg Config, n int) (*World, []*Body) {
	w := NewWorld(Vec2{0, -10}, cfg)
	anchor := NewStaticBody(0, Vec2{0, float64(n)})
	w.AddBody(anchor)
	bodies := []*Body{anchor}
	prev := anchor
	prevAnchor := Vec2{0, 0}
	for i := 1; i <= n; i++ {
		b := NewBody(i, Vec2{0, float64(n - i)}, 1, 0.1)
		if err := w.AddBody(b); err != nil {
			panic(err)
		}
		j := &Revolute{
			jointBase:    jointBase{A: prev, B: b},
			LocalAnchorA: prevAnchor,
			LocalAnchorB: Vec2{0, 0.5},
		}
		if i > 1 {
			j.LocalAnchorA = Vec2{0, -0.5}
		}
		if err := w.AddJoint(j); err != nil {
			panic(err)
		}
		prev = b
		prevAnchor = Vec2{0, -0.5}
		bodies = append(bodies, b)
	}
	return w, bodies
}

func allFinite(w *World) bool {
	for _, b := range w.Bodies() {
		if !finiteVec(b.Position) || !finite(b.Angle) ||
			!finiteVec(b.Velocity) || !finite(b.AngularVelocity) {
			return false
		}
	}
	return true
}

func TestChainStaysConnected(t *testing.T) {
	cfg := DefaultConfig()
	w, bodies := makeChain(cfg, 5)
	for i := 0; i < 600; i++ {
		if err := w.Step(dt); err != nil {
			t.Fatalf("step %d: %v", i, err)
		}
	}
	if !allFinite(w) {
		t.Fatal("non-finite state after 600 steps")
	}
	// Every joint anchor pair must remain coincident.
	for _, j := range w.Joints() {
		a, b := j.Bodies()
		rj := j.(*Revolute)
		_, pa := worldAnchor(a, rj.LocalAnchorA)
		_, pb := worldAnchor(b, rj.LocalAnchorB)
		if d := pa.Sub(pb).Len(); d > 0.02 {
			t.Fatalf("joint anchors separated by %v", d)
		}
	}
	_ = bodies
}

func TestChainSettlesAndSleeps(t *testing.T) {
	cfg := DefaultConfig()
	w, bodies := makeChain(cfg, 4)
	for i := 0; i < 1200; i++ {
		w.Step(dt)
	}
	for _, b := range bodies[1:] {
		if !b.Asleep {
			t.Fatalf("body %d not asleep after settling; v=%v w=%v",
				b.ID, b.Velocity, b.AngularVelocity)
		}
	}
	// Wake one body: the whole chain must wake.
	mid := bodies[2]
	w.ApplyImpulse(mid, Vec2{0.5, 0})
	for _, b := range bodies[1:] {
		if b.Asleep {
			t.Fatalf("body %d not woken with the chain", b.ID)
		}
	}
	// World keeps stepping fine afterwards.
	for i := 0; i < 120; i++ {
		if err := w.Step(dt); err != nil {
			t.Fatalf("step after wake: %v", err)
		}
	}
	if !allFinite(w) {
		t.Fatal("non-finite state after wake")
	}
}

func TestRevoluteLimitPullback(t *testing.T) {
	cfg := DefaultConfig()
	w := NewWorld(Vec2{0, -10}, cfg)
	anchor := NewStaticBody(0, Vec2{0, 0})
	w.AddBody(anchor)
	b := NewBody(1, Vec2{1, 0}.Rotate(1.2), 1, 0.1) // start beyond upper limit
	b.Angle = 1.2
	w.AddBody(b)
	j := &Revolute{
		jointBase:    jointBase{A: anchor, B: b},
		LocalAnchorA: Vec2{0, 0},
		LocalAnchorB: Vec2{-1, 0},
		EnableLimit:  true,
		LowerAngle:   -0.5,
		UpperAngle:   0.5,
	}
	if err := w.AddJoint(j); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 300; i++ {
		if err := w.Step(dt); err != nil {
			t.Fatalf("step %d: %v", i, err)
		}
		rel := b.Angle - anchor.Angle
		// Transient overshoot within one step is pulled back, never unbounded.
		if i > 30 && (rel > 0.5+0.15 || rel < -0.5-0.15) {
			t.Fatalf("step %d: angle %v escaped limits", i, rel)
		}
		if i >= 240 && (rel > 0.5+0.03 || rel < -0.5-0.03) {
			t.Fatalf("step %d: angle %v not pulled back into limits", i, rel)
		}
	}
	if !allFinite(w) {
		t.Fatal("non-finite state")
	}
}

func TestPrismaticLimit(t *testing.T) {
	cfg := DefaultConfig()
	w := NewWorld(Vec2{0, 0}, cfg)
	anchor := NewStaticBody(0, Vec2{0, 0})
	w.AddBody(anchor)
	b := NewBody(1, Vec2{2.5, 0}, 1, 0.1) // beyond upper translation 1
	w.AddBody(b)
	j := &Prismatic{
		jointBase:        jointBase{A: anchor, B: b},
		LocalAnchorA:     Vec2{0, 0},
		LocalAnchorB:     Vec2{0, 0},
		LocalAxisA:       Vec2{1, 0},
		EnableLimit:      true,
		LowerTranslation: 0,
		UpperTranslation: 1,
	}
	if err := w.AddJoint(j); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 120; i++ {
		w.Step(dt)
	}
	if b.Position.X > 1+0.02 || b.Position.X < -0.02 {
		t.Fatalf("translation %v outside limits", b.Position.X)
	}
	if math.Abs(b.Position.Y) > 0.02 {
		t.Fatalf("body drifted off axis: y=%v", b.Position.Y)
	}
}

func TestDistanceJointRange(t *testing.T) {
	cfg := DefaultConfig()
	w := NewWorld(Vec2{0, 0}, cfg)
	a := NewBody(1, Vec2{0, 0}, 1, 0.1)
	b := NewBody(2, Vec2{5, 0}, 1, 0.1) // too far: max is 2
	w.AddBody(a)
	w.AddBody(b)
	j := &Distance{
		jointBase:    jointBase{A: a, B: b},
		LocalAnchorA: Vec2{0, 0},
		LocalAnchorB: Vec2{0, 0},
		MinLength:    0.5,
		MaxLength:    2,
	}
	if err := w.AddJoint(j); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 120; i++ {
		w.Step(dt)
	}
	d := b.Position.Sub(a.Position).Len()
	if d > 2+0.05 {
		t.Fatalf("distance %v exceeds max", d)
	}
	// Now push them too close.
	a.Position, b.Position = Vec2{0, 0}, Vec2{0.1, 0}
	a.Velocity, b.Velocity = Vec2{}, Vec2{}
	w.WakeIsland(a)
	for i := 0; i < 120; i++ {
		w.Step(dt)
	}
	d = b.Position.Sub(a.Position).Len()
	if d < 0.5-0.05 {
		t.Fatalf("distance %v below min", d)
	}
}
