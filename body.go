package solver

// Body is a rigid body in 2D.
type Body struct {
	ID int

	Position Vec2
	Angle    float64

	Velocity        Vec2
	AngularVelocity float64

	Mass       float64
	InvMass    float64
	Inertia    float64
	InvInertia float64

	force  Vec2
	torque float64

	sleepTimer float64
	Asleep     bool
}

// NewBody creates a dynamic body with the given mass and rotational inertia.
func NewBody(id int, pos Vec2, mass, inertia float64) *Body {
	b := &Body{ID: id, Position: pos}
	b.SetMass(mass, inertia)
	return b
}

// NewStaticBody creates a body with infinite mass (immovable).
func NewStaticBody(id int, pos Vec2) *Body {
	return &Body{ID: id, Position: pos}
}

// SetMass sets mass properties; non-positive mass makes the body static.
func (b *Body) SetMass(mass, inertia float64) {
	if mass <= 0 {
		b.Mass, b.InvMass = 0, 0
		b.Inertia, b.InvInertia = 0, 0
		return
	}
	b.Mass, b.InvMass = mass, 1/mass
	if inertia <= 0 {
		b.Inertia, b.InvInertia = 0, 0
	} else {
		b.Inertia, b.InvInertia = inertia, 1/inertia
	}
}

// IsStatic reports whether the body is immovable.
func (b *Body) IsStatic() bool { return b.InvMass == 0 }

// ApplyForce adds a world-frame force (cleared after each step).
func (b *Body) ApplyForce(f Vec2) { b.force = b.force.Add(f) }

// ApplyTorque adds a torque (cleared after each step).
func (b *Body) ApplyTorque(t float64) { b.torque += t }
