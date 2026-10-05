package acsolver

import "testing"

func TestLongRunStability(t *testing.T) {
	c := DefaultConfig()
	c.RandomSeed = 12345
	w, _ := NewWorld(c)
	base, _ := w.CreateBody(BodyDef{Static: true, Position: Vec3{Y: 8}})
	prev := base
	const n = 8
	for i := 1; i <= n; i++ {
		b, _ := w.CreateBody(BodyDef{
			Position: Vec3{X: float64(i), Y: 8}, Mass: 1,
			InertiaDiag: Vec3{1, 1, 1},
		})
		w.CreateJoint(JointDef{Kind: JointDistance, BodyA: prev, BodyB: b, Distance: 1})
		prev = b
	}
	rng := NewRNG(12345)
	var failures int
	for step := 0; step < 10000; step++ {
		if step%13 == 0 {
			id := BodyID(1 + int(rng.Uint64()%n))
			w.ApplyForce(id, Vec3{rng.Range(-3, 3), -9.8, rng.Range(-1, 1)})
		}
		rep := w.Step()
		failures += len(rep.Failures)
	}
	if !allFinite(w) {
		t.Fatal("non-finite after 10k steps")
	}
	worst := 0.0
	for _, j := range w.Joints() {
		wa := worldAnchor(w.Body(j.BodyA()), j.def.AnchorA)
		wb := worldAnchor(w.Body(j.BodyB()), j.def.AnchorB)
		if d := absF(wb.Sub(wa).Length() - j.restDistance); d > worst {
			worst = d
		}
	}
	if worst > 0.05 {
		t.Fatalf("joint drift after 10k steps: %f, failures=%d", worst, failures)
	}
	t.Logf("10k steps ok, worst joint error=%.6f, failures=%d", worst, failures)
}

func TestSnapshotDeterministicBytes(t *testing.T) {
	w := buildReproWorld(t, false)
	runScenario(w, 30)
	s1, _ := w.Snapshot()
	s2, _ := w.Snapshot()
	if string(s1.data) != string(s2.data) {
		t.Fatal("identical state produced different snapshot bytes")
	}
}
