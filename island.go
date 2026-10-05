package acsolver

import "sort"

// island is an independently solvable connected component of dynamic bodies.
type island struct {
	id        int
	bodies    []int
	joints    []int
	timeAcc   float64
	timeScale float64
	stepCount int64
	failures  []*SolveFailure
}

// memberKey identifies an island by its smallest body index so clocks can be
// preserved across rebuilds (adding bodies/joints does not reset progress).
func (is *island) memberKey() int { return is.bodies[0] }

func buildIslands(bodies []*Body, joints []*Joint) []*island {
	n := len(bodies)
	parent := make([]int, n)
	rank := make([]int, n)
	for i := range parent {
		parent[i] = i
	}
	var find func(int) int
	find = func(x int) int {
		for parent[x] != x {
			parent[x] = parent[parent[x]]
			x = parent[x]
		}
		return x
	}
	union := func(a, b int) {
		ra, rb := find(a), find(b)
		if ra == rb {
			return
		}
		if rank[ra] < rank[rb] {
			ra, rb = rb, ra
		}
		parent[rb] = ra
		if rank[ra] == rank[rb] {
			rank[ra]++
		}
	}

	for _, j := range joints {
		a, b := int(j.def.BodyA), int(j.def.BodyB)
		if bodies[a].def.Static && bodies[b].def.Static {
			continue
		}
		if bodies[a].def.Static || bodies[b].def.Static {
			continue
		}
		union(a, b)
	}

	groups := map[int][]int{}
	for i, b := range bodies {
		if b.def.Static {
			continue
		}
		r := find(i)
		groups[r] = append(groups[r], i)
	}
	islands := make([]*island, 0, len(groups))
	for _, mem := range groups {
		sort.Ints(mem)
		islands = append(islands, &island{id: mem[0], bodies: mem, timeScale: 1})
	}
	sort.Slice(islands, func(a, b int) bool { return islands[a].id < islands[b].id })
	for idx, is := range islands {
		is.id = idx
	}

	rootOf := map[int]int{}
	for _, is := range islands {
		for _, bi := range is.bodies {
			rootOf[bi] = is.id
		}
	}
	for ji, jo := range joints {
		a, b := int(jo.def.BodyA), int(jo.def.BodyB)
		rid, ok := rootOf[a]
		if !ok {
			rid, ok = rootOf[b]
		}
		if !ok {
			continue
		}
		islands[rid].joints = append(islands[rid].joints, ji)
	}
	for _, is := range islands {
		sort.Ints(is.joints)
	}
	return islands
}

func (is *island) allSleeping(bodies []*Body) bool {
	for _, bi := range is.bodies {
		if !bodies[bi].sleeping {
			return false
		}
	}
	return true
}
