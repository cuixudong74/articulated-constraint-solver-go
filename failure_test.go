package acsolver

import (
	"errors"
	"math"
	"testing"
)

func TestInvalidParameters(t *testing.T) {
	if _, err := NewWorld(Config{}); !errors.Is(err, ErrInvalidParameter) {
		t.Fatalf("zero config should be invalid, got %v", err)
	}
	c := DefaultConfig()
	c.Relaxation = 5
	if _, err := NewWorld(c); !errors.Is(err, ErrInvalidParameter) {
		t.Fatalf("relaxation 5 must be invalid, got %v", err)
	}
	w := NewDefaultWorld()
	if _, err := w.CreateBody(BodyDef{Mass: -1}); !errors.Is(err, ErrInvalidParameter) {
		t.Fatalf("negative mass must be invalid, got %v", err)
	}
	if _, err := w.CreateBody(BodyDef{Mass: 1, Position: Vec3{X: math.NaN()}}); !errors.Is(err, ErrInvalidParameter) {
		t.Fatalf("NaN position must be invalid, got %v", err)
	}
	a, _ := w.CreateBody(BodyDef{Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	if _, err := w.CreateJoint(JointDef{Kind: JointRevolute, BodyA: a, BodyB: a}); !errors.Is(err, ErrSelfReferencingJoint) {
		t.Fatalf("self joint must fail, got %v", err)
	}
	b, _ := w.CreateBody(BodyDef{Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	if _, err := w.CreateJoint(JointDef{Kind: JointRevolute, BodyA: a, BodyB: b, Axis: Vec3{}}); !errors.Is(err, ErrInvalidParameter) {
		t.Fatalf("zero axis must fail, got %v", err)
	}
	if _, err := w.CreateJoint(JointDef{Kind: JointRevolute, BodyA: a, BodyB: BodyID(999)}); !errors.Is(err, ErrUnknownBody) {
		t.Fatalf("unknown body must fail, got %v", err)
	}
}

func TestConflictingDistanceJoints(t *testing.T) {
	w := NewDefaultWorld()
	a, _ := w.CreateBody(BodyDef{Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	b, _ := w.CreateBody(BodyDef{Position: Vec3{X: 2}, Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	if _, err := w.CreateJoint(JointDef{Kind: JointDistance, BodyA: a, BodyB: b, Distance: 2}); err != nil {
		t.Fatal(err)
	}
	_, err := w.CreateJoint(JointDef{Kind: JointDistance, BodyA: a, BodyB: b, Distance: 3})
	if !errors.Is(err, ErrConflictingJoint) {
		t.Fatalf("incompatible rigid distance joints must conflict, got %v", err)
	}
	// World must remain usable after the rejected creation.
	_ = w.Step()
}

func TestWorldStaysFiniteUnderExtremeImpulse(t *testing.T) {
	c := noSleepConfig()
	c.MaxLinearSpeed = 50
	c.MaxAngularSpeed = 50
	w, _ := NewWorld(c)
	base, _ := w.CreateBody(BodyDef{Static: true, Position: Vec3{Y: 3}})
	tip, _ := w.CreateBody(BodyDef{Position: Vec3{Y: 3}, Mass: 0.001, InertiaDiag: Vec3{0.001, 0.001, 0.001}})
	if _, err := w.CreateJoint(JointDef{
		Kind: JointDistance, BodyA: base, BodyB: tip, Distance: 0,
	}); err != nil {
		t.Fatal(err)
	}
	w.ApplyImpulse(tip, Vec3{1e6, 1e6, 0})
	for i := 0; i < 500; i++ {
		rep := w.Step()
		for _, f := range rep.Failures {
			if f.Kind == FailureNaNDetected {
				t.Fatalf("NaN failure at step %d", i)
			}
		}
		b := w.Body(tip)
		if !b.Position().IsFinite() || !b.LinearVelocity().IsFinite() {
			t.Fatalf("non-finite state at step %d: %+v", i, b.Position())
		}
	}
}

func TestEnergyDoesNotDiverge(t *testing.T) {
	c := noSleepConfig()
	w, _ := NewWorld(c)
	base, _ := w.CreateBody(BodyDef{Static: true})
	b1, _ := w.CreateBody(BodyDef{Position: Vec3{X: 1}, Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	w.CreateJoint(JointDef{Kind: JointDistance, BodyA: base, BodyB: b1, Distance: 1})
	// give a bounded initial energy
	w.Body(b1).SetLinearVelocity(Vec3{Y: 2})
	energy := func() float64 {
		v := w.Body(b1).LinearVelocity()
		h := w.Body(b1).Position().Y
		return 0.5*v.Length2() + 9.8*h
	}
	e0 := energy()
	var maxE float64 = e0
	for i := 0; i < 2000; i++ {
		w.Step()
		if e := energy(); e > maxE {
			maxE = e
		}
	}
	// total energy must not grow beyond a small XPBD numerical margin
	if maxE > e0*1.05+1e-6 {
		t.Fatalf("energy grew: e0=%f max=%f", e0, maxE)
	}
	// joint length must be preserved long-term
	p := w.Body(b1).Position()
	if d := p.Length(); math.Abs(d-1) > 0.02 {
		t.Fatalf("distance joint drifted: %f", d)
	}
}

func TestParamChangeKeepsState(t *testing.T) {
	w := buildReproWorld(t, false)
	w.Step()
	before := encodeState(w.State())
	c := w.Config()
	c.Iterations = 16
	c.Relaxation = 0.8
	if err := w.SetConfig(c); err != nil {
		t.Fatal(err)
	}
	after := encodeState(w.State())
	if string(before) != string(after) {
		t.Fatal("SetConfig must not mutate simulated state")
	}
	for i := 0; i < 100; i++ {
		w.Step()
	}
	if !allFinite(w) {
		t.Fatal("state became non-finite after parameter change")
	}
}
