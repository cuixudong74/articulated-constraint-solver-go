package acsolver

import (
	"encoding/binary"
	"fmt"
	"math"
)

// Snapshot is an opaque, deep-captured world state. Its binary layout is
// versioned and uses fixed-order, fixed-width little-endian encoding, so the
// same world always produces identical bytes on every platform/run.
type Snapshot struct {
	data []byte
}

const snapshotMagic = "ACSNAP01" // also encodes on-disk version 1

type snapWriter struct{ b []byte }

func (w *snapWriter) u64(v uint64) {
	var buf [8]byte
	binary.LittleEndian.PutUint64(buf[:], v)
	w.b = append(w.b, buf[:]...)
}
func (w *snapWriter) i64(v int64) { w.u64(uint64(v)) }
func (w *snapWriter) f64(v float64) {
	w.u64(math.Float64bits(v))
}
func (w *snapWriter) str(s string) {
	w.u64(uint64(len(s)))
	w.b = append(w.b, s...)
}
func (w *snapWriter) vec(v Vec3) { w.f64(v.X); w.f64(v.Y); w.f64(v.Z) }
func (w *snapWriter) quat(q Quat) {
	w.f64(q.X)
	w.f64(q.Y)
	w.f64(q.Z)
	w.f64(q.W)
}

type snapReader struct {
	b []byte
	o int
}

func (r *snapReader) u64() uint64 {
	if r.o+8 > len(r.b) {
		panic(snapCorrupt{})
	}
	v := binary.LittleEndian.Uint64(r.b[r.o:])
	r.o += 8
	return v
}
func (r *snapReader) i64() int64   { return int64(r.u64()) }
func (r *snapReader) f64() float64 { return math.Float64frombits(r.u64()) }
func (r *snapReader) str() string {
	n := int(r.u64())
	if r.o+n > len(r.b) {
		panic(snapCorrupt{})
	}
	s := string(r.b[r.o : r.o+n])
	r.o += n
	return s
}
func (r *snapReader) vec() Vec3 {
	return Vec3{r.f64(), r.f64(), r.f64()}
}
func (r *snapReader) quat() Quat {
	return Quat{r.f64(), r.f64(), r.f64(), r.f64()}
}

type snapCorrupt struct{}

func (snapCorrupt) Error() string { return "acsolver: corrupt or incompatible snapshot" }

// Snapshot captures the complete world state: configuration (excluding the
// parallelism flag, which does not affect results), all bodies with cached
// inverse mass/inertia and sleep timers, all joints with rest quantities,
// per-island clocks, id counters, elapsed time and RNG stream position.
func (w *World) Snapshot() (*Snapshot, error) {
	sw := &snapWriter{}
	sw.str(snapshotMagic)

	sw.f64(w.cfg.dtSeconds())
	sw.i64(int64(w.cfg.Iterations))
	sw.f64(w.cfg.Relaxation)
	sw.f64(w.cfg.Compliance)
	sw.f64(w.cfg.LinearSleepSpeed)
	sw.f64(w.cfg.AngularSleepSpeed)
	sw.i64(int64(w.cfg.SleepDelay))
	sw.f64(w.cfg.LinearDamping)
	sw.f64(w.cfg.AngularDamping)
	sw.f64(w.cfg.MaxLinearSpeed)
	sw.f64(w.cfg.MaxAngularSpeed)
	sw.u64(w.cfg.RandomSeed)

	sw.i64(int64(w.elapsed))
	sw.i64(int64(w.nextBody))
	sw.i64(int64(w.nextJoint))
	sw.u64(w.rng.state)
	sw.u64(w.rng.inc)

	sw.u64(uint64(len(w.bodies)))
	for _, b := range w.bodies {
		sw.vec(b.def.Position)
		sw.quat(b.def.Rotation)
		sw.f64(b.def.Mass)
		sw.vec(b.def.InertiaDiag)
		sw.vec(b.def.LinearVel)
		sw.vec(b.def.AngularVel)
		sw.vec(b.force)
		sw.vec(b.impulse)
		sw.vec(b.torque)
		sw.f64(b.invMass)
		sw.vec(b.invILoc)
		sw.f64(b.sleepT)
		sw.f64(b.def.TimeScale)
		if b.def.Static {
			sw.u64(1)
		} else {
			sw.u64(0)
		}
		if b.sleeping {
			sw.u64(1)
		} else {
			sw.u64(0)
		}
	}

	sw.u64(uint64(len(w.joints)))
	for _, j := range w.joints {
		sw.i64(int64(j.def.Kind))
		sw.i64(int64(j.def.BodyA))
		sw.i64(int64(j.def.BodyB))
		sw.vec(j.def.AnchorA)
		sw.vec(j.def.AnchorB)
		sw.vec(j.def.Axis)
		sw.f64(j.def.Distance)
		sw.f64(j.def.MinAngle)
		sw.f64(j.def.MaxAngle)
		sw.f64(j.def.MinDisplacement)
		sw.f64(j.def.MaxDisplacement)
		sw.bool64(j.def.HasAngleLimit)
		sw.bool64(j.def.HasDispLimit)
		sw.f64(j.def.Compliance)
		sw.f64(j.restDistance)
		sw.f64(j.baseDisplacement)
		sw.quat(j.restRel)
		sw.vec(j.pin)
		sw.f64(j.curAngle)
		sw.f64(j.curDisplacement)
		sw.bool64(j.enabled)
	}

	sw.u64(uint64(len(w.islands)))
	for _, is := range w.islands {
		sw.f64(is.timeAcc)
		sw.f64(is.timeScale)
		sw.i64(is.stepCount)
		sw.u64(uint64(len(is.bodies)))
		for _, bi := range is.bodies {
			sw.i64(int64(bi))
		}
		sw.u64(uint64(len(is.joints)))
		for _, ji := range is.joints {
			sw.i64(int64(ji))
		}
	}
	return &Snapshot{data: sw.b}, nil
}

// Restore replaces world state with the snapshot. The restored world is
// bit-identical in simulated values, so subsequent steps are frame-identical
// to an uninterrupted run.
func (w *World) Restore(s *Snapshot) error {
	if s == nil {
		return wrapInvalid("nil snapshot")
	}
	nw, err := decodeSnapshot(s.data, w.cfg.Parallel)
	if err != nil {
		return err
	}
	*w = *nw
	return nil
}

func decodeSnapshot(data []byte, parallel bool) (nw *World, err error) {
	defer func() {
		if rec := recover(); rec != nil {
			if sc, ok := rec.(snapCorrupt); ok {
				err = sc
				return
			}
			panic(rec)
		}
	}()
	r := &snapReader{b: data}
	if magic := r.str(); magic != snapshotMagic {
		return nil, snapCorrupt{}
	}

	cfg := Config{}
	cfg.TimeStep = timeFromSeconds(r.f64())
	cfg.Iterations = int(r.i64())
	cfg.Relaxation = r.f64()
	cfg.Compliance = r.f64()
	cfg.LinearSleepSpeed = r.f64()
	cfg.AngularSleepSpeed = r.f64()
	cfg.SleepDelay = durationNS(r.i64())
	cfg.LinearDamping = r.f64()
	cfg.AngularDamping = r.f64()
	cfg.MaxLinearSpeed = r.f64()
	cfg.MaxAngularSpeed = r.f64()
	cfg.RandomSeed = r.u64()
	cfg.Parallel = parallel
	if errv := cfg.validate(); errv != nil {
		return nil, fmt.Errorf("%w: snapshot config invalid", ErrInvalidParameter)
	}

	nw = &World{cfg: cfg}
	nw.elapsed = durationNS(r.i64())
	nw.nextBody = BodyID(r.i64())
	nw.nextJoint = JointID(r.i64())
	nw.rng = &RNG{state: r.u64(), inc: r.u64()}

	nb := int(r.u64())
	nw.bodies = make([]*Body, nb)
	for i := 0; i < nb; i++ {
		b := &Body{id: BodyID(i)}
		b.def.Position = r.vec()
		b.def.Rotation = r.quat()
		b.def.Mass = r.f64()
		b.def.InertiaDiag = r.vec()
		b.def.LinearVel = r.vec()
		b.def.AngularVel = r.vec()
		b.force = r.vec()
		b.impulse = r.vec()
		b.torque = r.vec()
		b.invMass = r.f64()
		b.invILoc = r.vec()
		b.sleepT = r.f64()
		b.def.TimeScale = r.f64()
		b.def.Static = r.u64() == 1
		b.sleeping = r.u64() == 1
		nw.bodies[i] = b
	}

	nj := int(r.u64())
	nw.joints = make([]*Joint, nj)
	for i := 0; i < nj; i++ {
		j := &Joint{id: JointID(i)}
		j.def.Kind = JointKind(r.i64())
		j.def.BodyA = BodyID(r.i64())
		j.def.BodyB = BodyID(r.i64())
		j.def.AnchorA = r.vec()
		j.def.AnchorB = r.vec()
		j.def.Axis = r.vec()
		j.def.Distance = r.f64()
		j.def.MinAngle = r.f64()
		j.def.MaxAngle = r.f64()
		j.def.MinDisplacement = r.f64()
		j.def.MaxDisplacement = r.f64()
		j.def.HasAngleLimit = r.u64() == 1
		j.def.HasDispLimit = r.u64() == 1
		j.def.Compliance = r.f64()
		j.restDistance = r.f64()
		j.baseDisplacement = r.f64()
		j.restRel = r.quat()
		j.pin = r.vec()
		j.curAngle = r.f64()
		j.curDisplacement = r.f64()
		j.enabled = r.u64() == 1
		nw.joints[i] = j
	}

	ni := int(r.u64())
	nw.islands = make([]*island, ni)
	nw.islandOf = make([]int, nb)
	for i := range nw.islandOf {
		nw.islandOf[i] = -1
	}
	for i := 0; i < ni; i++ {
		is := &island{id: i}
		is.timeAcc = r.f64()
		is.timeScale = r.f64()
		is.stepCount = r.i64()
		nm := int(r.u64())
		is.bodies = make([]int, nm)
		for k := 0; k < nm; k++ {
			is.bodies[k] = int(r.i64())
			nw.islandOf[is.bodies[k]] = i
		}
		nj2 := int(r.u64())
		is.joints = make([]int, nj2)
		for k := 0; k < nj2; k++ {
			is.joints[k] = int(r.i64())
		}
		nw.islands[i] = is
	}
	if r.o != len(r.b) {
		return nil, snapCorrupt{}
	}
	return nw, nil
}

func (w *snapWriter) bool64(v bool) {
	if v {
		w.u64(1)
	} else {
		w.u64(0)
	}
}
