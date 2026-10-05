package acsolver

import (
	"bytes"
	"math"
	"testing"
)

func buildReproWorld(t testing.TB, parallel bool) *World {
	c := noSleepConfig()
	c.Parallel = parallel
	w, err := NewWorld(c)
	if err != nil {
		t.Fatal(err)
	}
	base, _ := w.CreateBody(BodyDef{Static: true, Position: Vec3{Y: 5}})
	for i := 0; i < 4; i++ {
		_, err := w.CreateBody(BodyDef{
			Position:    Vec3{X: float64(i + 1), Y: 5},
			Mass:        1,
			InertiaDiag: Vec3{1, 1, 1},
		})
		if err != nil {
			t.Fatal(err)
		}
	}
	for i := 0; i < 4; i++ {
		a := BodyID(i)
		b := BodyID(i + 1)
		if i == 0 {
			a = base
		}
		j, err := w.CreateJoint(JointDef{Kind: JointDistance, BodyA: a, BodyB: b, Distance: 1})
		if err != nil {
			t.Fatal(err)
		}
		_ = j
	}
	// Deterministic perturbation sequence.
	rng := NewRNG(7)
	for i := 0; i < 10; i++ {
		id := BodyID(1 + int(rng.Uint64()%4))
		if err := w.ApplyImpulse(id, Vec3{rng.Range(-2, 2), rng.Range(-3, -1), rng.Range(-1, 1)}); err != nil {
			t.Fatal(err)
		}
		w.Step()
	}
	return w
}

func encodeState(st *WorldState) []byte {
	var buf bytes.Buffer
	writeF := func(v float64) {
		var b [8]byte
		u := math.Float64bits(v)
		for k := 0; k < 8; k++ {
			b[k] = byte(u >> (8 * k))
		}
		buf.Write(b[:])
	}
	writeF(float64(st.ElapsedNs))
	for _, b := range st.Bodies {
		for _, v := range b.Position {
			writeF(v)
		}
		for _, v := range b.Rotation {
			writeF(v)
		}
		for _, v := range b.LinearVel {
			writeF(v)
		}
		for _, v := range b.AngularVel {
			writeF(v)
		}
	}
	for _, j := range st.Joints {
		writeF(j.Angle)
		writeF(j.Displacement)
	}
	return buf.Bytes()
}

func runScenario(w *World, steps int) *WorldState {
	rng := NewRNG(99)
	for i := 0; i < steps; i++ {
		if i%7 == 0 {
			id := BodyID(1 + int(rng.Uint64()%4))
			w.ApplyForce(id, Vec3{rng.Range(-1, 1), -9.8 * 0.2, rng.Range(-1, 1)})
		}
		w.Step()
	}
	return w.State()
}

func TestSnapshotFrameIdentical(t *testing.T) {
	w := buildReproWorld(t, false)
	// Run 25 steps, snapshot, run 25 more, restore, run 25 more: must match.
	runScenario(w, 25)
	snap, err := w.Snapshot()
	if err != nil {
		t.Fatal(err)
	}
	afterA := runScenario(w, 25)

	if err := w.Restore(snap); err != nil {
		t.Fatal(err)
	}
	afterB := runScenario(w, 25)

	if !bytes.Equal(encodeState(afterA), encodeState(afterB)) {
		t.Fatal("restored run diverged from uninterrupted run")
	}
}

func TestReproducibleFromSeed(t *testing.T) {
	w1 := buildReproWorld(t, false)
	w2 := buildReproWorld(t, false)
	st1 := runScenario(w1, 60)
	st2 := runScenario(w2, 60)
	if !bytes.Equal(encodeState(st1), encodeState(st2)) {
		t.Fatal("same seed/input produced different outputs")
	}
}

func TestParallelSerialIdentical(t *testing.T) {
	// Two disconnected groups so parallel scheduling kicks in.
	build := func(parallel bool) *World {
		c := noSleepConfig()
		c.Parallel = parallel
		w, _ := NewWorld(c)
		for g := 0; g < 3; g++ {
			base, _ := w.CreateBody(BodyDef{Static: true, Position: Vec3{X: float64(g * 5), Y: 4}})
			b, _ := w.CreateBody(BodyDef{Position: Vec3{X: float64(g * 5), Y: 3}, Mass: 1, InertiaDiag: Vec3{1, 1, 1}})
			w.CreateJoint(JointDef{Kind: JointDistance, BodyA: base, BodyB: b, Distance: 1})
			w.ApplyImpulse(b, Vec3{float64(g + 1), -1, 0})
		}
		return w
	}
	ws := build(false)
	wp := build(true)
	if wp.Islands() != 3 {
		t.Fatalf("want 3 islands, got %d", wp.Islands())
	}
	var s1, s2 []byte
	for i := 0; i < 120; i++ {
		ws.Step()
		wp.Step()
		if i%20 == 19 {
			s1 = encodeState(ws.State())
			s2 = encodeState(wp.State())
			if !bytes.Equal(s1, s2) {
				t.Fatalf("parallel/serial differ at frame %d", i)
			}
		}
	}
}

func TestIslandPartitionStable(t *testing.T) {
	w := buildReproWorld(t, false)
	first := append([]int(nil), w.State().IslandOf...)
	w.Step()
	second := append([]int(nil), w.State().IslandOf...)
	if len(first) != len(second) {
		t.Fatal("island map changed size")
	}
	for i := range first {
		if first[i] != second[i] {
			t.Fatalf("island assignment changed at %d", i)
		}
	}
}
