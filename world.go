package solver

import (
	"math"
	"sync"
)

// World holds bodies and joints and advances the simulation.
type World struct {
	Gravity Vec2
	Config  Config

	bodies []*Body
	joints []Joint
	byID   map[int]*Body

	time   float64
	stepID uint64
}

// NewWorld creates a world with the given gravity and config.
func NewWorld(gravity Vec2, cfg Config) *World {
	cfg.normalize()
	return &World{Gravity: gravity, Config: cfg, byID: make(map[int]*Body)}
}

func finite(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }

func finiteVec(v Vec2) bool { return finite(v.X) && finite(v.Y) }

// AddBody registers a body. Returns error on duplicate ID or non-finite state.
func (w *World) AddBody(b *Body) error {
	if b == nil {
		return invalidParamf("nil body")
	}
	if !finiteVec(b.Position) || !finite(b.Angle) ||
		!finiteVec(b.Velocity) || !finite(b.AngularVelocity) {
		return invalidParamf("body %d has non-finite state", b.ID)
	}
	if b.Mass < 0 || b.Inertia < 0 {
		return invalidParamf("body %d has negative mass or inertia", b.ID)
	}
	if _, dup := w.byID[b.ID]; dup {
		return invalidParamf("duplicate body ID %d", b.ID)
	}
	w.byID[b.ID] = b
	w.bodies = append(w.bodies, b)
	return nil
}

// AddJoint registers a joint after validating its parameters.
func (w *World) AddJoint(j Joint) error {
	if j == nil {
		return invalidParamf("nil joint")
	}
	a, b := j.Bodies()
	if a == nil || b == nil {
		return invalidParamf("joint with nil body")
	}
	if w.byID[a.ID] != a || w.byID[b.ID] != b {
		return invalidParamf("joint references unregistered body")
	}
	if a.IsStatic() && b.IsStatic() {
		return &Error{Kind: ErrKindSingular,
			Message: "joint between two immovable bodies has no degrees of freedom"}
	}
	switch t := j.(type) {
	case *Revolute:
		if !finiteVec(t.LocalAnchorA) || !finiteVec(t.LocalAnchorB) || !finite(t.ReferenceAngle) {
			return invalidParamf("revolute joint has non-finite parameter")
		}
		if t.EnableLimit && !(t.LowerAngle <= t.UpperAngle) {
			return invalidParamf("revolute limit lower %v > upper %v", t.LowerAngle, t.UpperAngle)
		}
	case *Prismatic:
		if !finiteVec(t.LocalAnchorA) || !finiteVec(t.LocalAnchorB) || !finiteVec(t.LocalAxisA) {
			return invalidParamf("prismatic joint has non-finite parameter")
		}
		if t.LocalAxisA.LenSq() < 1e-12 {
			return invalidParamf("prismatic axis must be non-zero")
		}
		if t.EnableLimit && !(t.LowerTranslation <= t.UpperTranslation) {
			return invalidParamf("prismatic limit lower %v > upper %v", t.LowerTranslation, t.UpperTranslation)
		}
	case *Distance:
		if !finiteVec(t.LocalAnchorA) || !finiteVec(t.LocalAnchorB) {
			return invalidParamf("distance joint has non-finite parameter")
		}
		if !(t.MinLength <= t.MaxLength) || t.MinLength < 0 {
			return invalidParamf("distance length range [%v, %v] invalid", t.MinLength, t.MaxLength)
		}
	default:
		return invalidParamf("unknown joint type %T", j)
	}
	w.joints = append(w.joints, j)
	return nil
}

// Bodies returns the registered bodies in registration order.
func (w *World) Bodies() []*Body { return w.bodies }

// Joints returns the registered joints in registration order.
func (w *World) Joints() []Joint { return w.joints }

// Body returns the body with the given ID, or nil.
func (w *World) Body(id int) *Body { return w.byID[id] }

// Time returns accumulated simulation time.
func (w *World) Time() float64 { return w.time }

// Step advances the simulation by dt seconds. A non-nil error reports
// singular or conflicting constraints; the world remains steppable.
func (w *World) Step(dt float64) error {
	if !finite(dt) || dt <= 0 {
		return invalidParamf("invalid time step %v", dt)
	}
	// Clamp runtime-modified parameters into safe ranges so that mid-run
	// config changes cannot destabilize a converged state.
	w.Config.normalize()
	cfg := &w.Config

	// Integrate external forces into velocities (awake dynamic bodies only).
	for _, b := range w.bodies {
		if b.Asleep || b.IsStatic() {
			b.force, b.torque = Vec2{}, 0
			continue
		}
		b.Velocity = b.Velocity.Add(w.Gravity.Add(b.force.Scale(b.InvMass)).Scale(dt))
		b.AngularVelocity += dt * b.torque * b.InvInertia
	}

	islands := buildIslands(w.bodies, w.joints)

	// Solve islands in parallel; islands share no bodies so results are
	// deterministic regardless of scheduling.
	errs := make([]error, len(islands))
	converged := make([]bool, len(islands))
	work := func(i int) {
		errs[i] = w.solveIsland(&islands[i], dt, cfg)
		converged[i] = w.positionIsland(&islands[i], cfg)
		w.sleepIsland(&islands[i], dt, cfg)
	}
	workers := cfg.ParallelWorkers
	if workers > len(islands) {
		workers = len(islands)
	}
	if workers <= 1 {
		for i := range islands {
			work(i)
		}
	} else {
		var wg sync.WaitGroup
		queue := make(chan int, len(islands))
		for i := range islands {
			queue <- i
		}
		close(queue)
		for n := 0; n < workers; n++ {
			wg.Add(1)
			go func() {
				defer wg.Done()
				for i := range queue {
					work(i)
				}
			}()
		}
		wg.Wait()
	}

	// Clear forces.
	for _, b := range w.bodies {
		b.force, b.torque = Vec2{}, 0
	}

	w.time += dt
	w.stepID++

	// NaN guard: sanitize and report rather than corrupting the world.
	for _, b := range w.bodies {
		if !finiteVec(b.Position) || !finite(b.Angle) ||
			!finiteVec(b.Velocity) || !finite(b.AngularVelocity) {
			b.Position, b.Velocity = Vec2{}, Vec2{}
			b.Angle, b.AngularVelocity = 0, 0
			return invalidParamf("body %d produced non-finite state; velocities reset", b.ID)
		}
	}

	for _, err := range errs {
		if err != nil {
			return err
		}
	}
	for _, ok := range converged {
		if !ok {
			return &Error{Kind: ErrKindConflicting,
				Message: "position constraints did not fully converge (conflicting limits or over-constrained chain)"}
		}
	}
	return nil
}

// solveIsland runs velocity-level constraint solving and integrates positions.
func (w *World) solveIsland(is *island, dt float64, cfg *Config) error {
	var firstErr error
	active := is.joints[:0]
	for _, j := range is.joints {
		if err := j.initSolver(dt, cfg); err != nil {
			if firstErr == nil {
				firstErr = err
			}
			continue // skip broken joint, keep the rest solvable
		}
		active = append(active, j)
	}
	for _, j := range active {
		j.warmStart()
	}
	for it := 0; it < cfg.VelocityIterations; it++ {
		for _, j := range active {
			j.solveVelocity()
		}
	}
	for _, b := range is.bodies {
		b.Position = b.Position.Add(b.Velocity.Scale(dt))
		b.Angle += b.AngularVelocity * dt
	}
	return firstErr
}

// positionIsland runs position correction; reports whether constraints are
// satisfiable. Slow-but-steady convergence is fine; only residuals beyond a
// hard threshold count as conflicting (unsatisfiable) constraints.
func (w *World) positionIsland(is *island, cfg *Config) bool {
	for it := 0; it < cfg.PositionIterations; it++ {
		ok := true
		for _, j := range is.joints {
			if !j.solvePosition(cfg) {
				ok = false
			}
		}
		if ok {
			return true
		}
	}
	// Conflict verdict uses a loose tolerance so that merely slow convergence
	// (heavy chains, large steps) is not misreported as conflict.
	loose := *cfg
	loose.LinearSlop = 1.0
	loose.AngularSlop = 1.0
	for _, j := range is.joints {
		if !j.solvePosition(&loose) {
			return false
		}
	}
	return true
}

// sleepIsland updates sleep timers and puts rested islands to sleep.
func (w *World) sleepIsland(is *island, dt float64, cfg *Config) {
	if !cfg.EnableSleep {
		return
	}
	minTimer := math.Inf(1)
	lin2 := cfg.SleepLinearThreshold * cfg.SleepLinearThreshold
	for _, b := range is.bodies {
		if b.Velocity.LenSq() > lin2 || math.Abs(b.AngularVelocity) > cfg.SleepAngularThreshold {
			b.sleepTimer = 0
		} else {
			b.sleepTimer += dt
		}
		if b.sleepTimer < minTimer {
			minTimer = b.sleepTimer
		}
	}
	if minTimer >= cfg.SleepTime {
		for _, b := range is.bodies {
			b.Asleep = true
			b.Velocity = Vec2{}
			b.AngularVelocity = 0
		}
	}
}

// ApplyImpulse applies a linear impulse to a body and wakes its whole island.
func (w *World) ApplyImpulse(b *Body, imp Vec2) {
	if b == nil || b.IsStatic() || !finiteVec(imp) {
		return
	}
	w.WakeIsland(b)
	b.Velocity = b.Velocity.Add(imp.Scale(b.InvMass))
}

// WakeIsland wakes the body and every body connected to it via joints.
func (w *World) WakeIsland(b *Body) {
	if b == nil {
		return
	}
	// BFS over the joint graph.
	adj := make(map[*Body][]*Body)
	for _, j := range w.joints {
		x, y := j.Bodies()
		adj[x] = append(adj[x], y)
		adj[y] = append(adj[y], x)
	}
	stack := []*Body{b}
	seen := map[*Body]bool{b: true}
	for len(stack) > 0 {
		cur := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		cur.Asleep = false
		cur.sleepTimer = 0
		for _, nb := range adj[cur] {
			if !seen[nb] {
				seen[nb] = true
				stack = append(stack, nb)
			}
		}
	}
}
