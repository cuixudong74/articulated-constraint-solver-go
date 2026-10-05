package acsolver

// BodyState is the exportable runtime state of a body.
type BodyState struct {
	ID         BodyID
	Position   [3]float64
	Rotation   [4]float64 // X, Y, Z, W
	LinearVel  [3]float64
	AngularVel [3]float64
	Sleeping   bool
	Static     bool
}

// WorldState is a deterministic full-state export in ascending id order.
type WorldState struct {
	ElapsedNs int64
	Bodies    []BodyState
	Joints    []JointState
	IslandOf  []int
}

// State returns a copy of the current world state suitable for serialization
// or frame-by-frame comparison. Slices are always allocated (never nil) so
// output is stable across repeated calls.
func (w *World) State() *WorldState {
	st := &WorldState{
		ElapsedNs: int64(w.elapsed),
		Bodies:    make([]BodyState, 0, len(w.bodies)),
		Joints:    make([]JointState, 0, len(w.joints)),
		IslandOf:  make([]int, len(w.bodies)),
	}
	for i, b := range w.bodies {
		st.Bodies = append(st.Bodies, BodyState{
			ID:         b.id,
			Position:   [3]float64{b.def.Position.X, b.def.Position.Y, b.def.Position.Z},
			Rotation:   [4]float64{b.def.Rotation.X, b.def.Rotation.Y, b.def.Rotation.Z, b.def.Rotation.W},
			LinearVel:  [3]float64{b.def.LinearVel.X, b.def.LinearVel.Y, b.def.LinearVel.Z},
			AngularVel: [3]float64{b.def.AngularVel.X, b.def.AngularVel.Y, b.def.AngularVel.Z},
			Sleeping:   b.sleeping,
			Static:     b.def.Static,
		})
		if i < len(w.islandOf) {
			st.IslandOf[i] = w.islandOf[i]
		} else {
			st.IslandOf[i] = -1
		}
	}
	for _, j := range w.joints {
		st.Joints = append(st.Joints, JointState{
			ID:           j.id,
			Kind:         j.def.Kind,
			BodyA:        j.def.BodyA,
			BodyB:        j.def.BodyB,
			Angle:        j.curAngle,
			Displacement: j.curDisplacement,
		})
	}
	return st
}
