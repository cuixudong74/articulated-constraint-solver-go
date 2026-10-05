package acsolver

import "time"

func timeFromSeconds(s float64) time.Duration {
	return time.Duration(mathRound(s * float64(time.Second)))
}

func durationNS(ns int64) time.Duration { return time.Duration(ns) }

func mathRound(x float64) float64 {
	if x < 0 {
		return -mathRoundPos(-x)
	}
	return mathRoundPos(x)
}

func mathRoundPos(x float64) float64 {
	t := float64(int64(x))
	if x-t >= 0.5 {
		return t + 1
	}
	return t
}
