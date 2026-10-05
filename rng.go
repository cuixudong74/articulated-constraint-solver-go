package acsolver

import "math"

// RNG is a deterministic PCG-XSH-RR 64-bit-state / 32-bit-output pseudo
// random number generator. Two 32-bit outputs are combined for Uint64.
// The stream is fully reproducible across runs and platforms (pure integer
// arithmetic, no map iteration, no goroutine scheduling dependence).
type RNG struct {
	state uint64
	inc   uint64
}

// NewRNG creates a seeded generator. The seed undergoes the standard PCG
// initialization scheme.
func NewRNG(seed uint64) *RNG {
	r := &RNG{inc: 0x853c49e6748fea9b | 1}
	r.Seed(seed)
	return r
}

// Seed resets the generator state deterministically.
func (r *RNG) Seed(seed uint64) {
	r.state = 0
	r.pcg32()
	r.state += seed
	r.pcg32()
}

func (r *RNG) pcg32() uint32 {
	old := r.state
	r.state = old*6364136223846793005 + r.inc
	xorshifted := uint32(((old >> 18) ^ old) >> 27)
	rot := uint32(old >> 59)
	return (xorshifted >> rot) | (xorshifted << ((-rot) & 31))
}

// Uint64 returns a deterministic pseudo-random uint64.
func (r *RNG) Uint64() uint64 {
	hi := uint64(r.pcg32())
	lo := uint64(r.pcg32())
	return hi<<32 | lo
}

// Float64 returns a deterministic pseudo-random value in [0, 1).
// It uses 53 bits of mantissa and never returns NaN.
func (r *RNG) Float64() float64 {
	v := r.Uint64() >> 11 // 53 bits
	return float64(v) * (1.0 / float64(uint64(1)<<53))
}

// Range returns a deterministic pseudo-random value in [minVal, maxVal).
// Invalid ranges collapse to minVal instead of producing NaN.
func (r *RNG) Range(minVal, maxVal float64) float64 {
	if math.IsNaN(minVal) || math.IsNaN(maxVal) || minVal >= maxVal {
		return minVal
	}
	return minVal + r.Float64()*(maxVal-minVal)
}
