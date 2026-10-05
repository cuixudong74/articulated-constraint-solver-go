package solver

// Snapshot is a full copy of world dynamic state. Restoring a snapshot and
// re-running the same input sequence reproduces the original run exactly,
// because joint warm-start accumulators and sleep timers are included.
type Snapshot struct {
	bodies []bodyState
	joints []jointState
	time   float64
	stepID uint64
}

type bodyState struct {
	position        Vec2
	angle           float64
	velocity        Vec2
	angularVelocity float64
	sleepTimer      float64
	asleep          bool
}

// Snapshot captures the current world state.
func (w *World) Snapshot() *Snapshot {
	s := &Snapshot{
		bodies: make([]bodyState, len(w.bodies)),
		joints: make([]jointState, len(w.joints)),
		time:   w.time,
		stepID: w.stepID,
	}
	for i, b := range w.bodies {
		s.bodies[i] = bodyState{
			position:        b.Position,
			angle:           b.Angle,
			velocity:        b.Velocity,
			angularVelocity: b.AngularVelocity,
			sleepTimer:      b.sleepTimer,
			asleep:          b.Asleep,
		}
	}
	for i, j := range w.joints {
		s.joints[i] = j.cloneState()
	}
	return s
}

// Restore returns the world to a previously captured snapshot.
func (w *World) Restore(s *Snapshot) {
	if s == nil || len(s.bodies) != len(w.bodies) || len(s.joints) != len(w.joints) {
		return
	}
	for i, b := range w.bodies {
		st := s.bodies[i]
		b.Position = st.position
		b.Angle = st.angle
		b.Velocity = st.velocity
		b.AngularVelocity = st.angularVelocity
		b.sleepTimer = st.sleepTimer
		b.Asleep = st.asleep
		b.force, b.torque = Vec2{}, 0
	}
	for i, j := range w.joints {
		j.restoreState(s.joints[i])
	}
	w.time = s.time
	w.stepID = s.stepID
}
