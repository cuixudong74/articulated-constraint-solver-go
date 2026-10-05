package acsolver

import (
	"math"
	"testing"
	"time"
)

func noSleepConfig() Config {
	c := DefaultConfig()
	c.LinearDamping = 0
	c.AngularDamping = 0
	c.SleepDelay = 1<<62 - 1
	c.MaxLinearSpeed = 0
	c.MaxAngularSpeed = 0
	c.Parallel = false
	return c
}

func TestRevoluteAngleLimitPullBack(t *testing.T) {
	w, err := NewWorld(noSleepConfig())
	if err != nil {
		t.Fatal(err)
	}
	base, _ := w.CreateBody(BodyDef{Static: true})
	arm, _ := w.CreateBody(BodyDef{
		Position: Vec3{X: 1}, Mass: 1, InertiaDiag: Vec3{1, 1, 1},
	})
	// Hinge about Z, allowed range [-0.2, 0.2] rad.
	jid, err := w.CreateJoint(JointDef{
		Kind: JointRevolute, BodyA: base, BodyB: arm,
		Axis:          Vec3{Z: 1},
		AnchorB:       Vec3{X: 1},
		HasAngleLimit: true, MinAngle: -0.2, MaxAngle: 0.2,
	})
	if err != nil {
		t.Fatal(err)
	}
	// Kick the arm to rotate far past the upper limit.
	w.ApplyImpulse(arm, Vec3{Y: 8})
	maxA := math.Inf(-1)
	for i := 0; i < 300; i++ {
		w.Step()
		a := w.Joint(JointID(jid)).curAngle
		if a > maxA {
			maxA = a
		}
		if math.IsNaN(a) {
			t.Fatal("NaN angle")
		}
	}
	final := w.Joint(JointID(jid)).curAngle
	if final > 0.2+0.02 {
		t.Errorf("hinge exceeded limit: final=%f peak=%f", final, maxA)
	}
	if maxA > 0.2+0.3 {
		t.Errorf("hinge blew through limit before pull-back: peak=%f", maxA)
	}
}

func TestPrismaticDispLimitPullBack(t *testing.T) {
	w, _ := NewWorld(noSleepConfig())
	base, _ := w.CreateBody(BodyDef{Static: true})
	slider, _ := w.CreateBody(BodyDef{Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	jid, err := w.CreateJoint(JointDef{
		Kind: JointPrismatic, BodyA: base, BodyB: slider,
		Axis:         Vec3{X: 1},
		HasDispLimit: true, MinDisplacement: -0.5, MaxDisplacement: 0.5,
	})
	if err != nil {
		t.Fatal(err)
	}
	w.ApplyImpulse(slider, Vec3{X: 20})
	peak := math.Inf(-1)
	for i := 0; i < 400; i++ {
		w.Step()
		d := w.Joint(JointID(jid)).curDisplacement
		if d > peak {
			peak = d
		}
	}
	final := w.Joint(JointID(jid)).curDisplacement
	if final > 0.5+0.02 {
		t.Errorf("slider exceeded limit final=%f peak=%f", final, peak)
	}
	// perpendicular motion must be suppressed
	if p := w.Body(slider).Position(); math.Abs(p.Y)+math.Abs(p.Z) > 0.02 {
		t.Errorf("slider moved off axis: %+v", p)
	}
}

func TestSleepWakeChain(t *testing.T) {
	c := DefaultConfig()
	c.SleepDelay = 100 * time.Millisecond
	c.LinearSleepSpeed = 0.05
	c.LinearDamping = 0.8
	c.AngularDamping = 0.8
	w, _ := NewWorld(c)
	a, _ := w.CreateBody(BodyDef{Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	b, _ := w.CreateBody(BodyDef{Position: Vec3{X: 2}, Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	_, err := w.CreateJoint(JointDef{Kind: JointDistance, BodyA: a, BodyB: b, Distance: 2})
	if err != nil {
		t.Fatal(err)
	}
	// damp to rest and wait for sleep
	for i := 0; i < 200; i++ {
		w.Step()
	}
	if !w.Body(a).Sleeping() || !w.Body(b).Sleeping() {
		t.Fatalf("bodies did not sleep: a=%v b=%v", w.Body(a).Sleeping(), w.Body(b).Sleeping())
	}
	// Impulse on a must wake the whole chain.
	if err := w.ApplyImpulse(a, Vec3{X: 5}); err != nil {
		t.Fatal(err)
	}
	w.Step()
	if w.Body(a).Sleeping() || w.Body(b).Sleeping() {
		t.Fatalf("chain not woken: a=%v b=%v", w.Body(a).Sleeping(), w.Body(b).Sleeping())
	}
}
