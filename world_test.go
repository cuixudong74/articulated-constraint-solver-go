package solver

import (
	"math"
	"testing"
)

// makeTree builds a small tree: static root, two branches of revolute links
// plus one prismatic slider, exercising mixed joint types.
func makeTree(cfg Config) *World {
	w := NewWorld(Vec2{0, -10}, cfg)
	root := NewStaticBody(0, Vec2{0, 5})
	w.AddBody(root)
	mk := func(id int, pos Vec2) *Body {
		b := NewBody(id, pos, 1, 0.1)
		if err := w.AddBody(b); err != nil {
			panic(err)
		}
		return b
	}
	a1 := mk(1, Vec2{-0.5, 4.5})
	a2 := mk(2, Vec2{-0.5, 3.5})
	b1 := mk(3, Vec2{0.5, 4.5})
	c1 := mk(4, Vec2{0, 6})
	rev := func(a, b *Body, la, lb Vec2, ref float64) {
		j := &Revolute{jointBase: jointBase{A: a, B: b},
			LocalAnchorA: la, LocalAnchorB: lb, ReferenceAngle: ref,
			EnableLimit: true, LowerAngle: -1.5, UpperAngle: 1.5}
		if err := w.AddJoint(j); err != nil {
			panic(err)
		}
	}
	rev(root, a1, Vec2{-0.5, 0}, Vec2{0, 0.5}, 0)
	rev(a1, a2, Vec2{0, -0.5}, Vec2{0, 0.5}, 0)
	rev(root, b1, Vec2{0.5, 0}, Vec2{0, 0.5}, 0)
	if err := w.AddJoint(&Prismatic{
		jointBase: jointBase{A: root, B: c1}, LocalAxisA: Vec2{0, 1},
		EnableLimit: true, LowerTranslation: 0.2, UpperTranslation: 2,
	}); err != nil {
		panic(err)
	}
	if err := w.AddJoint(&Distance{
		jointBase: jointBase{A: a2, B: b1}, MinLength: 0.5, MaxLength: 3,
	}); err != nil {
		panic(err)
	}
	return w
}

// drive applies a deterministic input sequence (fixed "random" pattern).
func drive(w *World, step int) {
	if step%17 == 3 {
		w.ApplyImpulse(w.Body(2), Vec2{0.3, 0.2})
	}
	if step%23 == 5 {
		w.ApplyImpulse(w.Body(3), Vec2{-0.2, 0.1})
	}
	if step%29 == 7 {
		w.Body(4).ApplyForce(Vec2{0, 30})
	}
}

func statesEqual(a, b *World) bool {
	ba, bb := a.Bodies(), b.Bodies()
	if len(ba) != len(bb) {
		return false
	}
	for i := range ba {
		if ba[i].Position != bb[i].Position || ba[i].Angle != bb[i].Angle ||
			ba[i].Velocity != bb[i].Velocity ||
			ba[i].AngularVelocity != bb[i].AngularVelocity ||
			ba[i].Asleep != bb[i].Asleep {
			return false
		}
	}
	return true
}

func TestSnapshotRestoreExact(t *testing.T) {
	cfg := DefaultConfig()
	w := makeTree(cfg)
	for i := 0; i < 100; i++ {
		drive(w, i)
		w.Step(dt)
	}
	snap := w.Snapshot()
	// Record 100 frames of continued run.
	type frame struct {
		pos   []Vec2
		angle []float64
	}
	record := func() []frame {
		var out []frame
		for i := 100; i < 200; i++ {
			drive(w, i)
			w.Step(dt)
			f := frame{}
			for _, b := range w.Bodies() {
				f.pos = append(f.pos, b.Position)
				f.angle = append(f.angle, b.Angle)
			}
			out = append(out, f)
		}
		return out
	}
	run1 := record()
	w.Restore(snap)
	run2 := record()
	if len(run1) != len(run2) {
		t.Fatal("frame count mismatch")
	}
	for i := range run1 {
		for k := range run1[i].pos {
			if run1[i].pos[k] != run2[i].pos[k] || run1[i].angle[k] != run2[i].angle[k] {
				t.Fatalf("frame %d body %d diverged after restore: %v vs %v",
					i, k, run1[i].pos[k], run2[i].pos[k])
			}
		}
	}
}

func TestDeterminismParallelVsSerial(t *testing.T) {
	// Several disconnected islands to exercise parallel solving.
	build := func(workers int) *World {
		cfg := DefaultConfig()
		cfg.ParallelWorkers = workers
		w := NewWorld(Vec2{0, -10}, cfg)
		id := 0
		for g := 0; g < 6; g++ {
			x := float64(g) * 10
			anchor := NewStaticBody(id, Vec2{x, 5})
			id++
			w.AddBody(anchor)
			prev := anchor
			for k := 0; k < 4; k++ {
				b := NewBody(id, Vec2{x, 4.5 - float64(k)}, 1, 0.1)
				id++
				w.AddBody(b)
				j := &Revolute{jointBase: jointBase{A: prev, B: b},
					LocalAnchorA: Vec2{0, -0.5}, LocalAnchorB: Vec2{0, 0.5}}
				if prev == anchor {
					j.LocalAnchorA = Vec2{0, 0}
				}
				w.AddJoint(j)
				prev = b
			}
		}
		return w
	}
	w1 := build(1)
	w2 := build(4)
	for i := 0; i < 300; i++ {
		if i == 50 {
			// Same impulse sequence on both worlds.
			for _, w := range []*World{w1, w2} {
				w.ApplyImpulse(w.Body(3), Vec2{1, 0.5})
				w.ApplyImpulse(w.Body(17), Vec2{-0.5, 1})
			}
		}
		w1.Step(dt)
		w2.Step(dt)
	}
	if !statesEqual(w1, w2) {
		t.Fatal("parallel and serial runs diverged")
	}
	// Reproducibility: a third identical serial run matches exactly.
	w3 := build(1)
	for i := 0; i < 300; i++ {
		if i == 50 {
			w3.ApplyImpulse(w3.Body(3), Vec2{1, 0.5})
			w3.ApplyImpulse(w3.Body(17), Vec2{-0.5, 1})
		}
		w3.Step(dt)
	}
	if !statesEqual(w1, w3) {
		t.Fatal("identical runs diverged")
	}
}

func TestConflictingConstraints(t *testing.T) {
	cfg := DefaultConfig()
	w := NewWorld(Vec2{0, 0}, cfg)
	a := NewStaticBody(0, Vec2{0, 0})
	b := NewStaticBody(1, Vec2{10, 0})
	mid := NewBody(2, Vec2{5, 0}, 1, 0.1)
	w.AddBody(a)
	w.AddBody(b)
	w.AddBody(mid)
	// Both distance joints cannot be satisfied simultaneously.
	w.AddJoint(&Distance{jointBase: jointBase{A: a, B: mid}, MaxLength: 1})
	w.AddJoint(&Distance{jointBase: jointBase{A: b, B: mid}, MaxLength: 1})
	var sawConflict bool
	for i := 0; i < 60; i++ {
		err := w.Step(dt)
		if err != nil {
			se, ok := err.(*Error)
			if !ok {
				t.Fatalf("unexpected error type: %v", err)
			}
			if se.Kind == ErrKindConflicting {
				sawConflict = true
			}
		}
	}
	if !sawConflict {
		t.Fatal("expected conflicting-constraints error")
	}
	if !allFinite(w) {
		t.Fatal("non-finite state after conflict")
	}
	// World remains steppable.
	for i := 0; i < 60; i++ {
		w.Step(dt)
	}
	if !allFinite(w) {
		t.Fatal("non-finite state after continued stepping")
	}
}

func TestSingularAndInvalidParams(t *testing.T) {
	cfg := DefaultConfig()
	w := NewWorld(Vec2{0, -10}, cfg)
	s1 := NewStaticBody(0, Vec2{0, 0})
	s2 := NewStaticBody(1, Vec2{1, 0})
	w.AddBody(s1)
	w.AddBody(s2)
	// Revolute between two immovable bodies is singular.
	err := w.AddJoint(&Revolute{jointBase: jointBase{A: s1, B: s2}})
	se, ok := err.(*Error)
	if !ok || se.Kind != ErrKindSingular {
		t.Fatalf("expected singular error, got %v", err)
	}

	// Invalid parameters are rejected at the API boundary.
	if err := w.AddBody(NewBody(0, Vec2{}, 1, 1)); err == nil {
		t.Fatal("expected duplicate ID error")
	}
	bad := NewBody(2, Vec2{math.NaN(), 0}, 1, 1)
	if err := w.AddBody(bad); err == nil {
		t.Fatal("expected NaN rejection")
	}
	if err := w.Step(0); err == nil {
		t.Fatal("expected invalid dt error")
	}
	if err := w.Step(math.NaN()); err == nil {
		t.Fatal("expected NaN dt error")
	}
	w2 := NewWorld(Vec2{}, cfg)
	x := NewBody(0, Vec2{}, 1, 1)
	y := NewBody(1, Vec2{1, 0}, 1, 1)
	w2.AddBody(x)
	w2.AddBody(y)
	if err := w2.AddJoint(&Prismatic{jointBase: jointBase{A: x, B: y}}); err == nil {
		t.Fatal("expected zero-axis rejection")
	}
	if err := w2.AddJoint(&Distance{jointBase: jointBase{A: x, B: y},
		MinLength: 3, MaxLength: 1}); err == nil {
		t.Fatal("expected bad range rejection")
	}
	if err := w2.AddJoint(&Revolute{jointBase: jointBase{A: x, B: y},
		EnableLimit: true, LowerAngle: 1, UpperAngle: -1}); err == nil {
		t.Fatal("expected bad angle limit rejection")
	}
}

func TestEnergyDoesNotGrow(t *testing.T) {
	cfg := DefaultConfig()
	cfg.EnableSleep = false
	w := NewWorld(Vec2{0, 0}, cfg) // no gravity, no damping
	anchor := NewStaticBody(0, Vec2{0, 0})
	w.AddBody(anchor)
	b := NewBody(1, Vec2{1, 0}, 1, 0.1)
	w.AddBody(b)
	w.AddJoint(&Revolute{jointBase: jointBase{A: anchor, B: b},
		LocalAnchorB: Vec2{-1, 0}})
	b.AngularVelocity = 2
	energy := func() float64 {
		return 0.5*b.Mass*b.Velocity.LenSq() + 0.5*b.Inertia*b.AngularVelocity*b.AngularVelocity
	}
	e0 := energy()
	maxRatio := 1.0
	for i := 0; i < 6000; i++ { // 100 simulated seconds
		if err := w.Step(dt); err != nil {
			t.Fatalf("step %d: %v", i, err)
		}
		if e := energy(); e/e0 > maxRatio {
			maxRatio = e / e0
		}
	}
	if maxRatio > 1.01 {
		t.Fatalf("energy grew by factor %v", maxRatio)
	}
	if !allFinite(w) {
		t.Fatal("non-finite state")
	}
}

func TestConfigChangeKeepsStability(t *testing.T) {
	cfg := DefaultConfig()
	w, _ := makeChain(cfg, 4)
	for i := 0; i < 600; i++ {
		w.Step(dt)
	}
	// Converged; now change solver parameters mid-run.
	w.Config.VelocityIterations = 4
	w.Config.PositionIterations = 2
	w.Config.Relaxation = 0.1
	w.Config.normalize()
	for i := 0; i < 600; i++ {
		if err := w.Step(dt); err != nil {
			t.Fatalf("step %d after config change: %v", i, err)
		}
	}
	if !allFinite(w) {
		t.Fatal("non-finite state after config change")
	}
	for _, j := range w.Joints() {
		a, b := j.Bodies()
		rj := j.(*Revolute)
		_, pa := worldAnchor(a, rj.LocalAnchorA)
		_, pb := worldAnchor(b, rj.LocalAnchorB)
		if d := pa.Sub(pb).Len(); d > 0.1 {
			t.Fatalf("chain fell apart after config change: %v", d)
		}
	}
}
