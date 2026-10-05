package acsolver

import "testing"

func TestIndependentIslandSpeeds(t *testing.T) {
	build := func(scale float64) *World {
		c := noSleepConfig()
		w, _ := NewWorld(c)
		a, _ := w.CreateBody(BodyDef{Static: true})
		// free body connected only to a static anchor -> its own island
		b, _ := w.CreateBody(BodyDef{
			Position: Vec3{Y: -5}, Mass: 1, InertiaDiag: Vec3{1, 1, 1},
			TimeScale: scale,
		})
		w.CreateJoint(JointDef{Kind: JointDistance, BodyA: a, BodyB: b, Distance: 5})
		w.ApplyImpulse(b, Vec3{X: 2})
		return w
	}
	w1 := build(1)
	w05 := build(0.5)
	for i := 0; i < 100; i++ {
		w1.Step()
		w05.Step()
	}
	// w05 accumulates half the simulated time: its stepCount is lower.
	var s1, s2 int64
	for _, is := range w1.islands {
		s1 += is.stepCount
	}
	for _, is := range w05.islands {
		s2 += is.stepCount
	}
	if s2 >= s1 {
		t.Fatalf("slower island ran as many steps: slow=%d fast=%d", s2, s1)
	}
	// The two independent groups in one world must not block each other.
	c := noSleepConfig()
	c.Parallel = true
	w, _ := NewWorld(c)
	baseA, _ := w.CreateBody(BodyDef{Static: true, Position: Vec3{X: -10}})
	baseB, _ := w.CreateBody(BodyDef{Static: true, Position: Vec3{X: 10}})
	fa, _ := w.CreateBody(BodyDef{Position: Vec3{X: -10, Y: -3}, Mass: 1, InertiaDiag: Vec3{1, 1, 1}, TimeScale: 0.25})
	fb, _ := w.CreateBody(BodyDef{Position: Vec3{X: 10, Y: -3}, Mass: 1, InertiaDiag: Vec3{1, 1, 1}, TimeScale: 1})
	w.CreateJoint(JointDef{Kind: JointDistance, BodyA: baseA, BodyB: fa, Distance: 3})
	w.CreateJoint(JointDef{Kind: JointDistance, BodyA: baseB, BodyB: fb, Distance: 3})
	var fastSteps, slowSteps int64
	for i := 0; i < 200; i++ {
		rep := w.Step()
		_ = rep
	}
	for _, is := range w.islands {
		switch w.bodies[is.bodies[0]].def.TimeScale {
		case 0.25:
			slowSteps = is.stepCount
		default:
			fastSteps = is.stepCount
		}
	}
	if fastSteps <= slowSteps {
		t.Fatalf("fast island did not outrun slow: fast=%d slow=%d", fastSteps, slowSteps)
	}
}

func TestWokenReport(t *testing.T) {
	c := DefaultConfig()
	c.SleepDelay = 0
	w, _ := NewWorld(c)
	a, _ := w.CreateBody(BodyDef{Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	b, _ := w.CreateBody(BodyDef{Position: Vec3{X: 2}, Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
	w.CreateJoint(JointDef{Kind: JointDistance, BodyA: a, BodyB: b, Distance: 2})
	for i := 0; i < 5; i++ {
		w.Step()
	}
	if !w.Body(a).Sleeping() || !w.Body(b).Sleeping() {
		t.Fatal("expected both asleep")
	}
	w.ApplyImpulse(a, Vec3{X: 1})
	rep := w.Step()
	found := map[int]bool{}
	for _, id := range rep.Woken {
		found[id] = true
	}
	if !found[0] || !found[1] {
		t.Fatalf("woken report missing chain members: %v", rep.Woken)
	}
}
