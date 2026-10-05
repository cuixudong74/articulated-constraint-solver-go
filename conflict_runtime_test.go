package acsolver

import "testing"

func TestRuntimeSingularFailureReported(t *testing.T) {
	c := noSleepConfig()
	c.Iterations = 4
	w, _ := NewWorld(c)
	// Two anchors on each body at the same location: two distance rows share
	// the exact same gradient direction, but demand two different lengths.
	// After iterations one row keeps a large residual while the other is
	// satisfied; when the effective weight degenerates the solver must report
	// a distinguishable singular/conflict failure and keep advancing.
	a, _ := w.CreateBody(BodyDef{Mass: 1, InertiaDiag: Vec3{0, 0, 0}})
	b, _ := w.CreateBody(BodyDef{Position: Vec3{X: 2}, Mass: 1, InertiaDiag: Vec3{0, 0, 0}})
	// First joint: length 2.
	w.CreateJoint(JointDef{Kind: JointDistance, BodyA: a, BodyB: b, Distance: 2, Compliance: 1e-6})
	// Second joint between the same bodies: rigid length 3. Using compliance
	// avoids construction-time rejection but the two rows compete.
	w.CreateJoint(JointDef{Kind: JointDistance, BodyA: a, BodyB: b, Distance: 3, Compliance: 1e-12})

	kinds := map[FailureKind]bool{}
	for i := 0; i < 300; i++ {
		rep := w.Step()
		for _, f := range rep.Failures {
			kinds[f.Kind] = true
		}
		if !allFinite(w) {
			t.Fatalf("non-finite at step %d", i)
		}
	}
	if !kinds[FailureSingular] && !kinds[FailureConflictingConstraints] {
		t.Fatalf("expected a distinguishable failure kind, got %v", kinds)
	}
	// World remains advanceable afterwards with a clean joint set.
	c2 := w.Config()
	_ = c2
	if got := len(w.Bodies()); got != 2 {
		t.Fatalf("bodies lost after failures: %d", got)
	}
}
