package acsolver

import (
	"math"
	"testing"
)

func TestSmokeDistanceChain(t *testing.T) {
	w := NewDefaultWorld()
	cfg := w.Config()
	cfg.LinearDamping = 0
	cfg.AngularDamping = 0
	cfg.SleepDelay = 1 << 62
	cfg.MaxLinearSpeed = 0
	cfg.MaxAngularSpeed = 0
	if err := w.SetConfig(cfg); err != nil {
		t.Fatal(err)
	}

	anchor, _ := w.CreateBody(BodyDef{Static: true, Position: Vec3{0, 10, 0}})
	b1, _ := w.CreateBody(BodyDef{Position: Vec3{0, 10, 0}, Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	b2, _ := w.CreateBody(BodyDef{Position: Vec3{0, 9, 0}, Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	if _, err := w.CreateJoint(JointDef{Kind: JointDistance, BodyA: anchor, BodyB: b1, Distance: 0}); err != nil {
		t.Fatal(err)
	}
	if _, err := w.CreateJoint(JointDef{Kind: JointDistance, BodyA: b1, BodyB: b2, Distance: 1}); err != nil {
		t.Fatal(err)
	}

	w.ApplyForce(b2, Vec3{10, 0, 0})
	for i := 0; i < 200; i++ {
		rep := w.Step()
		for _, f := range rep.Failures {
			t.Errorf("failure: %v", f)
		}
	}
	p0 := w.Body(b1).Position()
	p1 := w.Body(b2).Position()
	if d := p0.Sub(Vec3{0, 10, 0}).Length(); d > 0.1 {
		t.Errorf("b1 drifted from anchor: %v d=%f", p0, d)
	}
	if d := p1.Sub(p0).Length(); math.Abs(d-1) > 0.05 {
		t.Errorf("chain length violated: %f", d)
	}
	if !allFinite(w) {
		t.Error("non-finite state")
	}
}

func allFinite(w *World) bool {
	for _, b := range w.Bodies() {
		if !b.Position().IsFinite() || !b.Rotation().IsFinite() ||
			!b.LinearVelocity().IsFinite() || !b.AngularVelocity().IsFinite() {
			return false
		}
	}
	return true
}
