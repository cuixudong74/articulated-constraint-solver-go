package acsolver

import (
	"errors"
	"sync"
	"time"
)

// World holds all bodies, joints and solver state.
type World struct {
	cfg       Config
	bodies    []*Body
	joints    []*Joint
	islands   []*island
	islandOf  []int
	rng       *RNG
	elapsed   time.Duration
	nextBody  BodyID
	nextJoint JointID
	lastWoken []int
}

// NewWorld creates an empty world with the given configuration.
func NewWorld(cfg Config) (*World, error) {
	if err := cfg.validate(); err != nil {
		return nil, err
	}
	return &World{
		cfg:      cfg,
		rng:      NewRNG(cfg.RandomSeed),
		islandOf: nil,
	}, nil
}

// NewDefaultWorld creates an empty world with DefaultConfig.
func NewDefaultWorld() *World {
	w, _ := NewWorld(DefaultConfig())
	return w
}

func (w *World) Config() Config { return w.cfg }

// SetConfig replaces solver parameters after validation. Parameter changes do
// not reset bodies, joints or clocks; XPBD converges from current poses.
func (w *World) SetConfig(c Config) error {
	if err := c.validate(); err != nil {
		return err
	}
	w.cfg = c
	w.rng.Seed(c.RandomSeed)
	return nil
}

func (w *World) CreateBody(def BodyDef) (BodyID, error) {
	id := w.nextBody
	b, err := newBody(id, def)
	if err != nil {
		return 0, err
	}
	w.bodies = append(w.bodies, b)
	w.nextBody++
	w.rebuildIslands()
	return id, nil
}

func (w *World) CreateJoint(def JointDef) (JointID, error) {
	if int(def.BodyA) >= len(w.bodies) || int(def.BodyB) >= len(w.bodies) ||
		def.BodyA < 0 || def.BodyB < 0 {
		return 0, ErrUnknownBody
	}
	a, b := w.bodies[int(def.BodyA)], w.bodies[int(def.BodyB)]
	if !a.def.Static && !b.def.Static && a.def.TimeScale != b.def.TimeScale {
		return 0, wrapInvalid("bodies connected by a joint must share TimeScale")
	}
	// Reject a duplicate incompatible joint on the same ordered/unordered pair.
	if err := w.checkJointConflict(def); err != nil {
		return 0, err
	}
	id := w.nextJoint
	j, err := newJoint(id, def, a, b)
	if err != nil {
		return 0, err
	}
	w.joints = append(w.joints, j)
	w.nextJoint++
	w.rebuildIslands()
	w.wakeChain(int(def.BodyA))
	w.wakeChain(int(def.BodyB))
	return id, nil
}

func (w *World) checkJointConflict(def JointDef) error {
	key := func(i, j BodyID) (BodyID, BodyID) {
		if i > j {
			return j, i
		}
		return i, j
	}
	x, y := key(def.BodyA, def.BodyB)
	for _, ex := range w.joints {
		px, py := key(ex.def.BodyA, ex.def.BodyB)
		if px != x || py != y {
			continue
		}
		// Two rigid (near-zero compliance) distance joints with different
		// rest lengths on the same pair are unsatisfiable.
		if ex.def.Kind == JointDistance && def.Kind == JointDistance &&
			ex.def.Compliance <= 1e-12 && def.Compliance <= 1e-12 {
			la := ex.restDistance
			lb := def.Distance
			if lb <= 0 {
				wa := worldAnchor(w.bodies[int(def.BodyA)], def.AnchorA)
				wb := worldAnchor(w.bodies[int(def.BodyB)], def.AnchorB)
				lb = wb.Sub(wa).Length()
			}
			if absF(la-lb) > 1e-6 {
				return errors.Join(ErrConflictingJoint,
					wrapInvalid("two rigid distance joints with different lengths on same bodies"))
			}
		}
	}
	return nil
}

func (w *World) Body(id BodyID) *Body {
	if int(id) < 0 || int(id) >= len(w.bodies) {
		return nil
	}
	return w.bodies[int(id)]
}

func (w *World) Joint(id JointID) *Joint {
	if int(id) < 0 || int(id) >= len(w.joints) {
		return nil
	}
	return w.joints[int(id)]
}

func (w *World) Bodies() []*Body  { return w.bodies }
func (w *World) Joints() []*Joint { return w.joints }
func (w *World) Islands() int     { return len(w.islands) }
func (w *World) IslandOf(id BodyID) int {
	if int(id) < 0 || int(id) >= len(w.islandOf) {
		return -1
	}
	return w.islandOf[int(id)]
}

func (w *World) requireBody(id BodyID) (*Body, error) {
	b := w.Body(id)
	if b == nil {
		return nil, ErrUnknownBody
	}
	return b, nil
}

// ApplyForce applies a world-space force and wakes the connected chain.
func (w *World) ApplyForce(id BodyID, f Vec3) error {
	b, err := w.requireBody(id)
	if err != nil {
		return err
	}
	woken := w.wakeChain(int(id))
	b.ApplyForce(f)
	w.lastWoken = append(w.lastWoken, woken...)
	return nil
}

// ApplyImpulse applies a world-space impulse and wakes the connected chain.
func (w *World) ApplyImpulse(id BodyID, j Vec3) error {
	b, err := w.requireBody(id)
	if err != nil {
		return err
	}
	woken := w.wakeChain(int(id))
	b.ApplyImpulse(j)
	w.lastWoken = append(w.lastWoken, woken...)
	return nil
}

// Wake wakes a body and every body reachable from it through joints.
func (w *World) Wake(id BodyID) error {
	if _, err := w.requireBody(id); err != nil {
		return err
	}
	w.lastWoken = append(w.lastWoken, w.wakeChain(int(id))...)
	return nil
}

func (w *World) Elapsed() time.Duration { return w.elapsed }

// rebuildIslands recomputes connected components deterministically and tries
// to preserve per-island clocks keyed by the smallest member body.
func (w *World) rebuildIslands() {
	prev := map[int]*island{}
	for _, is := range w.islands {
		prev[is.memberKey()] = is
	}
	fresh := buildIslands(w.bodies, w.joints)
	for _, is := range fresh {
		if old, ok := prev[is.memberKey()]; ok {
			is.timeAcc = old.timeAcc
			is.stepCount = old.stepCount
			is.failures = old.failures
		}
		is.timeScale = w.bodies[is.bodies[0]].def.TimeScale
	}
	w.islands = fresh
	w.islandOf = make([]int, len(w.bodies))
	for i := range w.islandOf {
		w.islandOf[i] = -1
	}
	for idx, is := range w.islands {
		for _, bi := range is.bodies {
			w.islandOf[bi] = idx
		}
	}
}

// wakeChain marks every body reachable via joints from seed as awake and
// returns the ascending list of body indices that transitioned to awake.
func (w *World) wakeChain(seed int) []int {
	adj := make([][]int, len(w.bodies))
	for _, j := range w.joints {
		if !j.enabled {
			continue
		}
		a, b := int(j.def.BodyA), int(j.def.BodyB)
		adj[a] = append(adj[a], b)
		adj[b] = append(adj[b], a)
	}
	seen := map[int]bool{seed: true}
	stack := []int{seed}
	var woken []int
	for len(stack) > 0 {
		x := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		if !w.bodies[x].sleeping && !w.bodies[x].def.Static {
			// already awake; still traverse, but only record transitions
		}
		if w.bodies[x].sleeping {
			woken = append(woken, x)
		}
		w.bodies[x].Wake()
		neighbors := append([]int(nil), adj[x]...)
		sortInts(neighbors)
		for _, n := range neighbors {
			if !seen[n] {
				seen[n] = true
				stack = append(stack, n)
			}
		}
	}
	sortInts(woken)
	return woken
}

// Step advances the world by one fixed reference TimeStep. Each island
// consumes TimeStep * TimeScale worth of its own clock; islands never block
// each other. The report aggregates results in ascending island order.
func (w *World) Step() *StepReport {
	dt := w.cfg.dtSeconds()
	report := &StepReport{Dt: dt, Iterations: w.cfg.Iterations}
	// Consume one deterministic RNG value per reference step so the stream
	// position advances identically regardless of parallel island execution.
	_ = w.rng.Uint64()
	if len(w.lastWoken) > 0 {
		report.Woken = append(report.Woken, w.lastWoken...)
		w.lastWoken = nil
	}

	type result struct {
		idx   int
		is    *island
		steps int
		fail  *SolveFailure
	}
	results := make([]result, len(w.islands))

	run := func(idx int) result {
		is := w.islands[idx]
		steps := 0
		var fail *SolveFailure
		if !is.allSleeping(w.bodies) {
			is.timeAcc += dt * is.timeScale
			// Run complete fixed sub-steps; remainder stays in timeAcc.
			for is.timeAcc+1e-12 >= dt {
				f := solveIsland(is, dt, w.cfg, w.bodies, w.joints)
				if f != nil {
					fail = f
					break
				}
				is.timeAcc -= dt
				is.stepCount++
				steps++
			}
		}
		return result{idx, is, steps, fail}
	}

	if w.cfg.Parallel && len(w.islands) > 1 {
		var wg sync.WaitGroup
		for i := range w.islands {
			wg.Add(1)
			go func(i int) {
				defer wg.Done()
				results[i] = run(i)
			}(i)
		}
		wg.Wait()
	} else {
		for i := range w.islands {
			results[i] = run(i)
		}
	}

	// Aggregate deterministically.
	for _, r := range results {
		if r.steps > 0 {
			report.Stepped = append(report.Stepped, r.is.id)
		}
		if r.fail != nil {
			report.Failures = append(report.Failures, r.fail)
		}
	}
	w.updateSleeping(dt, report)
	w.elapsed += w.cfg.TimeStep
	return report
}

func (w *World) updateSleeping(dt float64, report *StepReport) {
	lin := w.cfg.LinearSleepSpeed
	ang := w.cfg.AngularSleepSpeed
	delay := w.cfg.SleepDelay.Seconds()
	for _, is := range w.islands {
		// An island sleeps only when every member is slow (joint stability).
		slow := true
		for _, bi := range is.bodies {
			b := w.bodies[bi]
			if b.sleeping {
				continue
			}
			if b.def.LinearVel.Length() > lin || b.def.AngularVel.Length() > ang {
				slow = false
			}
		}
		if !slow {
			for _, bi := range is.bodies {
				w.bodies[bi].sleepT = 0
			}
			continue
		}
		for _, bi := range is.bodies {
			b := w.bodies[bi]
			if b.sleeping {
				continue
			}
			b.sleepT += dt
			if b.sleepT >= delay {
				b.PutToSleep()
				report.Slept = append(report.Slept, int(b.id))
			}
		}
	}
}

// StepReport summarizes one Step call.
type StepReport struct {
	Dt         float64
	Stepped    []int
	Slept      []int
	Woken      []int
	Failures   []*SolveFailure
	Iterations int
}

func absF(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}

func sortInts(s []int) {
	for i := 1; i < len(s); i++ {
		for j := i; j > 0 && s[j-1] > s[j]; j-- {
			s[j-1], s[j] = s[j], s[j-1]
		}
	}
}
