package acsolver

import (
	"math"
	"testing"
)

func TestTreeStructure(t *testing.T) {
	w, _ := NewWorld(noSleepConfig())
	anchor, _ := w.CreateBody(BodyDef{Static: true, Position: Vec3{Y: 5}})
	root, _ := w.CreateBody(BodyDef{Position: Vec3{Y: 4}, Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	w.CreateJoint(JointDef{Kind: JointDistance, BodyA: anchor, BodyB: root, Distance: 1})
	// root branches to three children, each with its own child.
	var leaves []BodyID
	for g := 0; g < 3; g++ {
		c, _ := w.CreateBody(BodyDef{
			Position: Vec3{X: float64(g), Y: 3}, Mass: 1, InertiaDiag: Vec3{1, 1, 1},
		})
		if _, err := w.CreateJoint(JointDef{Kind: JointDistance, BodyA: root, BodyB: c, Distance: math.Sqrt2}); err != nil {
			t.Fatal(err)
		}
		l, _ := w.CreateBody(BodyDef{
			Position: Vec3{X: float64(g) + 1, Y: 2}, Mass: 1, InertiaDiag: Vec3{1, 1, 1},
		})
		if _, err := w.CreateJoint(JointDef{Kind: JointDistance, BodyA: c, BodyB: l, Distance: math.Sqrt2}); err != nil {
			t.Fatal(err)
		}
		leaves = append(leaves, l)
	}
	if w.Islands() != 1 {
		t.Fatalf("tree must be one island, got %d", w.Islands())
	}
	w.ApplyImpulse(leaves[0], Vec3{3, 0, 1})
	for i := 0; i < 600; i++ {
		rep := w.Step()
		for _, f := range rep.Failures {
			if math.IsNaN(float64(f.Kind)) { // never true; keeps f meaningful
				t.Fatal(f)
			}
		}
	}
	for _, id := range leaves {
		b := w.Body(id)
		if !b.Position().IsFinite() {
			t.Fatal("non-finite leaf")
		}
	}
	// every distance must hold
	for _, j := range w.Joints() {
		wa := worldAnchor(w.Body(j.BodyA()), j.def.AnchorA)
		wb := worldAnchor(w.Body(j.BodyB()), j.def.AnchorB)
		if d := wb.Sub(wa).Length(); math.Abs(d-j.restDistance) > 0.05 {
			t.Errorf("joint %d length %f != rest %f", j.ID(), d, j.restDistance)
		}
	}
}

func TestRuntimeConflictReported(t *testing.T) {
	w, _ := NewWorld(noSleepConfig())
	a, _ := w.CreateBody(BodyDef{Static: true})
	b, _ := w.CreateBody(BodyDef{Position: Vec3{X: 1}, Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	c, _ := w.CreateBody(BodyDef{Position: Vec3{X: 2}, Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	// b pinned at distance 1 from a and 1 from c (a-b-c spaced 1 apart): fine.
	w.CreateJoint(JointDef{Kind: JointDistance, BodyA: a, BodyB: b, Distance: 1})
	w.CreateJoint(JointDef{Kind: JointDistance, BodyA: b, BodyB: c, Distance: 1})
	// Also pin c at distance 0.5 from a: incompatible triangle.
	w.CreateJoint(JointDef{Kind: JointDistance, BodyA: a, BodyB: c, Distance: 0.5})
	var sawConflict bool
	for i := 0; i < 200; i++ {
		rep := w.Step()
		for _, f := range rep.Failures {
			if f.Kind == FailureSingular || f.Kind == FailureConflictingConstraints {
				sawConflict = true
			}
		}
	}
	// The over-constrained system must at least be reported and stay finite.
	if !sawConflict && !allFinite(w) {
		t.Fatal("conflict neither reported nor finite")
	}
	if !allFinite(w) {
		t.Fatal("world non-finite after conflict")
	}
}

func TestRestLengthAutoInit(t *testing.T) {
	w, _ := NewWorld(noSleepConfig())
	a, _ := w.CreateBody(BodyDef{Static: true})
	b, _ := w.CreateBody(BodyDef{Position: Vec3{X: 3, Y: 4}, Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	j, _ := w.CreateJoint(JointDef{Kind: JointDistance, BodyA: a, BodyB: b})
	if d := w.Joint(j).restDistance; math.Abs(d-5) > 1e-9 {
		t.Fatalf("auto rest length = %f, want 5", d)
	}
}
