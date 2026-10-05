package solver

import "sort"

// island is a connected group of awake bodies and the joints between them.
type island struct {
	bodies []*Body
	joints []Joint
}

// buildIslands partitions dynamic bodies into connected components linked by
// joints. Sleep/wake is component-granular: a component containing at least
// one awake body is solved as a whole (sleeping members are woken), fully
// asleep components are skipped. Bodies, joints and islands are
// deterministically ordered so identical inputs always yield identical
// partitions.
func buildIslands(bodies []*Body, joints []Joint) []island {
	index := make(map[*Body]int, len(bodies))
	var dyn []*Body
	for _, b := range bodies {
		if b.IsStatic() {
			continue
		}
		index[b] = len(dyn)
		dyn = append(dyn, b)
	}
	if len(dyn) == 0 {
		return nil
	}

	parent := make([]int, len(dyn))
	for i := range parent {
		parent[i] = i
	}
	var find func(int) int
	find = func(i int) int {
		for parent[i] != i {
			parent[i] = parent[parent[i]]
			i = parent[i]
		}
		return i
	}
	union := func(a, b int) {
		ra, rb := find(a), find(b)
		if ra != rb {
			parent[rb] = ra
		}
	}

	type jrec struct {
		j   Joint
		idx int
	}
	var jrecs []jrec
	for idx, j := range joints {
		a, b := j.Bodies()
		ia, oka := index[a]
		ib, okb := index[b]
		if !oka && !okb {
			continue // joint between static bodies: nothing to solve
		}
		if oka && okb {
			union(ia, ib)
		}
		jrecs = append(jrecs, jrec{j, idx})
	}

	// Component awake iff any member is awake.
	awakeRoot := make(map[int]bool)
	for i, b := range dyn {
		if !b.Asleep {
			awakeRoot[find(i)] = true
		}
	}

	bodiesOf := make(map[int][]*Body)
	for i, b := range dyn {
		r := find(i)
		if !awakeRoot[r] {
			continue
		}
		// Wake sleeping members of an awake component.
		if b.Asleep {
			b.Asleep = false
			b.sleepTimer = 0
		}
		bodiesOf[r] = append(bodiesOf[r], b)
	}

	jointsOf := make(map[int][]Joint)
	for _, rec := range jrecs {
		a, b := rec.j.Bodies()
		r := -1
		if ia, ok := index[a]; ok {
			r = find(ia)
		} else if ib, ok := index[b]; ok {
			r = find(ib)
		}
		if r < 0 || !awakeRoot[r] {
			continue
		}
		jointsOf[r] = append(jointsOf[r], rec.j)
	}

	roots := make([]int, 0, len(bodiesOf))
	for r := range bodiesOf {
		roots = append(roots, r)
	}
	sort.Ints(roots)

	islands := make([]island, 0, len(roots))
	for _, r := range roots {
		bs := bodiesOf[r]
		sort.Slice(bs, func(i, k int) bool { return bs[i].ID < bs[k].ID })
		islands = append(islands, island{bodies: bs, joints: jointsOf[r]})
	}
	sort.Slice(islands, func(i, k int) bool {
		return islands[i].bodies[0].ID < islands[k].bodies[0].ID
	})
	return islands
}
