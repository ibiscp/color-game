import heapq
import time
from ortools.graph.python import linear_sum_assignment
from pdb import get_pdb


def manhattan(pos, target_pos, n):
    return abs(pos // n - target_pos // n) + abs(pos % n - target_pos % n)


# ── OR-Tools optimal assignment ───────────────────────────────────────────────

def optimal_assignment(board_ids: list, target_colors: list, n: int) -> dict:
    """
    Use OR-Tools linear_sum_assignment to find the optimal mapping
    from tile IDs to target positions, minimizing total Manhattan distance.
    Returns {tile_id: target_position_index}.
    """
    tile_positions: dict[str, list] = {}
    for pos, tile_id in enumerate(board_ids):
        if tile_id is None:
            continue
        color = tile_id.rsplit("-", 1)[0]
        tile_positions.setdefault(color, []).append((tile_id, pos))

    target_by_color: dict[str, list[int]] = {}
    for pos, color in enumerate(target_colors):
        if color is None:
            continue
        target_by_color.setdefault(color, []).append(pos)

    assignment: dict[str, int] = {}
    for color, tiles in tile_positions.items():
        targets = target_by_color.get(color, [])
        if len(tiles) == 1 and len(targets) == 1:
            assignment[tiles[0][0]] = targets[0]
            continue

        solver = linear_sum_assignment.SimpleLinearSumAssignment()
        for i, (tile_id, pos) in enumerate(tiles):
            for j, tgt in enumerate(targets):
                cost = manhattan(pos, tgt, n)
                solver.add_arc_with_cost(i, j, cost)

        status = solver.solve()
        if status == linear_sum_assignment.SimpleLinearSumAssignment.OPTIMAL:
            for i in range(len(tiles)):
                j = solver.right_mate(i)
                assignment[tiles[i][0]] = targets[j]
        else:
            for i, (tile_id, pos) in enumerate(tiles):
                assignment[tile_id] = targets[i % len(targets)]

    return assignment


# ── Parity / solvability ──────────────────────────────────────────────────────

def _count_inversions(seq: list) -> int:
    inv = 0
    for i in range(len(seq)):
        for j in range(i + 1, len(seq)):
            if seq[i] > seq[j]:
                inv += 1
    return inv


def _is_solvable(board_ids: list, assignment: dict, n: int) -> bool:
    goal = [None] * (n * n)
    for tile_id, pos in assignment.items():
        goal[pos] = tile_id

    goal_tiles_ordered = [t for t in goal if t is not None]
    goal_order = {tile_id: i for i, tile_id in enumerate(goal_tiles_ordered)}

    board_tiles = [t for t in board_ids if t is not None]
    board_in_goal_order = [goal_order[t] for t in board_tiles]

    inv = _count_inversions(board_in_goal_order)

    if n % 2 == 1:
        return inv % 2 == 0
    else:
        blank_start_row = n - 1 - board_ids.index(None) // n
        blank_goal_row = n - 1 - goal.index(None) // n
        return (inv + blank_start_row + blank_goal_row) % 2 == 0


def _fix_assignment_parity(board_ids: list, assignment: dict, target_colors: list, n: int) -> dict:
    target_by_color: dict[str, list[int]] = {}
    for pos, color in enumerate(target_colors):
        if color is None:
            continue
        target_by_color.setdefault(color, []).append(pos)

    tile_pos = {tid: pos for pos, tid in enumerate(board_ids) if tid is not None}
    best_swap = None
    best_extra = float("inf")

    for color, targets in target_by_color.items():
        if len(targets) < 2:
            continue
        color_tiles = [(tid, assignment[tid]) for tid in assignment if tid.rsplit("-", 1)[0] == color]
        if len(color_tiles) < 2:
            continue

        for i in range(len(color_tiles)):
            for j in range(i + 1, len(color_tiles)):
                ta, ga = color_tiles[i]
                tb, gb = color_tiles[j]
                pa, pb = tile_pos[ta], tile_pos[tb]
                original = manhattan(pa, ga, n) + manhattan(pb, gb, n)
                swapped  = manhattan(pa, gb, n) + manhattan(pb, ga, n)
                extra = swapped - original
                if extra < best_extra:
                    best_extra = extra
                    best_swap = (ta, tb, ga, gb)

    if best_swap:
        ta, tb, ga, gb = best_swap
        new = dict(assignment)
        new[ta] = gb
        new[tb] = ga
        return new
    return assignment


def ensure_solvable_assignment(board_ids: list, assignment: dict, target_colors: list, n: int) -> dict:
    if _is_solvable(board_ids, assignment, n):
        return assignment
    return _fix_assignment_parity(board_ids, assignment, target_colors, n)


# ── Grid helpers ──────────────────────────────────────────────────────────────

def _neighbors(pos, n):
    r, c = divmod(pos, n)
    result = []
    if r > 0:    result.append(pos - n)
    if r < n-1:  result.append(pos + n)
    if c > 0:    result.append(pos - 1)
    if c < n-1:  result.append(pos + 1)
    return result


# ── Heuristics ────────────────────────────────────────────────────────────────

def _md_lc(state: tuple, assignment: dict, n: int) -> int:
    """Manhattan Distance + Linear Conflict (admissible fallback for n=3)."""
    total = 0
    for pos, tile_id in enumerate(state):
        if tile_id is None:
            continue
        tgt = assignment.get(tile_id)
        if tgt is None:
            continue
        total += manhattan(pos, tgt, n)

    for r in range(n):
        items = []
        for c in range(n):
            tid = state[r * n + c]
            if tid is None:
                continue
            tgt = assignment.get(tid)
            if tgt is not None and tgt // n == r:
                items.append((c, tgt % n))
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                if items[i][1] > items[j][1]:
                    total += 2

    for c in range(n):
        items = []
        for r in range(n):
            tid = state[r * n + c]
            if tid is None:
                continue
            tgt = assignment.get(tid)
            if tgt is not None and tgt % n == c:
                items.append((r, tgt // n))
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                if items[i][1] > items[j][1]:
                    total += 2

    return total


# ── A* solver (works for all sizes) ──────────────────────────────────────────

def _reconstruct(came_from, state):
    path = []
    while came_from[state] is not None:
        prev_state, move = came_from[state]
        path.append(move)
        state = prev_state
    path.reverse()
    return path


def astar_solve(board_ids: list, assignment: dict, n: int, timeout: float) -> list[int] | None:
    """
    A* with PDB heuristic for n≥4, MD+LC for n=3.
    Returns list of board position indices to move (same format as JS solver).
    """
    start = tuple(board_ids)
    blank = start.index(None)

    # Derive blank_target and load PDB
    assigned_positions = set(assignment.values())
    blank_target = (set(range(n * n)) - assigned_positions).pop()

    if n >= 4:
        pdb = get_pdb(n, blank_target)
        def h(state):
            return pdb.heuristic(state, assignment)
    else:
        def h(state):
            return _md_lc(state, assignment, n)

    # Goal check: every tile at its target position
    goal_set = {(pos, tid) for tid, pos in assignment.items()}
    def is_goal(state):
        return all(state[pos] == tid for tid, pos in assignment.items())

    if is_goal(start):
        return []

    counter = 0
    h0 = h(start)
    open_heap = [(h0, counter, 0, start, blank)]
    came_from: dict = {start: None}
    g_score: dict = {start: 0}
    deadline = time.time() + timeout

    while open_heap:
        if time.time() > deadline:
            return None

        f, _, g, state, blank_pos = heapq.heappop(open_heap)

        if g_score.get(state, float("inf")) < g:
            continue

        if is_goal(state):
            return _reconstruct(came_from, state)

        for nb in _neighbors(blank_pos, n):
            new_state = list(state)
            new_state[blank_pos] = new_state[nb]
            new_state[nb] = None
            new_state_t = tuple(new_state)
            ng = g + 1

            if ng < g_score.get(new_state_t, float("inf")):
                g_score[new_state_t] = ng
                came_from[new_state_t] = (state, nb)
                nh = h(new_state_t)
                counter += 1
                heapq.heappush(open_heap, (ng + nh, counter, ng, new_state_t, nb))

    return None


# ── Constructive fallback (for very hard 5×5 cases) ──────────────────────────

def _bfs_move(n, state_list, blank, target_tiles, frozen):
    goal_positions = {tid: tgt for tid, tgt in target_tiles}
    tiles_only = [tid for tid, _ in target_tiles]

    def get_positions(state):
        return tuple(state.index(tid) for tid in tiles_only)

    def goal_reached(positions):
        return all(positions[i] == goal_positions[tiles_only[i]] for i in range(len(tiles_only)))

    start_positions = get_positions(state_list)
    if goal_reached(start_positions):
        return []

    start_key = start_positions + (blank,)
    prev = {start_key: None}
    queue = [(start_positions, blank)]
    head = 0

    while head < len(queue):
        positions, b = queue[head]
        head += 1
        pk = positions + (b,)

        for nb in _neighbors(b, n):
            if nb in frozen:
                continue
            new_pos = list(positions)
            for i, pos in enumerate(positions):
                if pos == nb:
                    new_pos[i] = b
            new_pos_t = tuple(new_pos)
            nk = new_pos_t + (nb,)
            if nk in prev:
                continue
            prev[nk] = (pk, nb)
            if goal_reached(new_pos_t):
                seq = []
                k = nk
                while prev[k] is not None:
                    pk2, move = prev[k]
                    seq.append(move)
                    k = pk2
                seq.reverse()
                return seq
            queue.append((new_pos_t, nb))

    return None


def _cleanup_moves(start_ids, moves):
    seq = [start_ids.index(None)] + moves
    changed = True
    while changed:
        changed = False
        res = [seq[0]]
        for i in range(1, len(seq)):
            if len(res) >= 2 and res[-2] == seq[i]:
                res.pop()
                changed = True
            else:
                res.append(seq[i])
        seq = res
    return seq[1:]


def constructive_solve(board_ids: list, assignment: dict, n: int) -> list[int]:
    """Row-by-row constructive solver — always finds a solution quickly."""
    state = list(board_ids)
    blank = state.index(None)
    out = []
    frozen = set()

    tile_for_cell = {v: k for k, v in assignment.items()}

    def slide(cell):
        nonlocal blank
        state[blank] = state[cell]
        state[cell] = None
        blank = cell
        out.append(cell)

    def solve_group(pairs):
        pairs = [(tid, tgt) for tid, tgt in pairs if tid is not None]
        if not pairs:
            return
        if all(state[tgt] == tid for tid, tgt in pairs):
            for _, tgt in pairs:
                frozen.add(tgt)
            return
        moves = _bfs_move(n, state, blank, pairs, frozen)
        if moves:
            for cell in moves:
                slide(cell)
        for _, tgt in pairs:
            frozen.add(tgt)

    def peel_line(cells):
        pairs = [(tile_for_cell.get(c), c) for c in cells]
        pairs = [(tid, tgt) for tid, tgt in pairs if tid is not None]
        if len(pairs) <= 2:
            solve_group(pairs)
            return
        for tid, tgt in pairs[:-2]:
            solve_group([(tid, tgt)])
        solve_group(pairs[-2:])

    blank_target = next((i for i in range(n*n) if tile_for_cell.get(i) is None), n*n - 1)
    bt_r, bt_c = divmod(blank_target, n)
    fr = min(bt_r, n - 2)
    fc = min(bt_c, n - 2)

    r0, r1, c0, c1 = 0, n-1, 0, n-1

    while r1 - r0 > 1:
        if r0 < fr:
            peel_line([r0*n + c for c in range(c0, c1+1)])
            r0 += 1
        else:
            peel_line([r1*n + c for c in range(c0, c1+1)])
            r1 -= 1

    while c1 - c0 > 1:
        if c0 < fc:
            peel_line([r*n + c0 for r in range(r0, r1+1)])
            c0 += 1
        else:
            peel_line([r*n + c1 for r in range(r0, r1+1)])
            c1 -= 1

    corner = [r0*n+c0, r0*n+c1, r1*n+c0, r1*n+c1]
    pairs = [(tile_for_cell.get(c), c) for c in corner]
    pairs = [(tid, tgt) for tid, tgt in pairs if tid is not None]
    solve_group(pairs)

    return _cleanup_moves(board_ids, out)


# ── OR-Tools CP-SAT solver ────────────────────────────────────────────────────

def cpsat_solve(board_ids: list, assignment: dict, n: int,
                T_max: int, timeout: float) -> list[int] | None:
    """
    Use OR-Tools CP-SAT to find a solution of exactly T_max moves.
    Returns the move list if SAT within timeout, else None.

    The model:
      - blank_pos[t]  : IntVar, position of blank at step t
      - tile_pos[t][i]: IntVar, position of tile i at step t
      - moved[t][i]   : BoolVar, True iff tile i moves at step t
    Constraints: initial state, goal state, valid transitions (adjacent),
                 AllDifferent per step, conditional tile movements.
    """
    from ortools.sat.python import cp_model

    tiles = [tid for tid in board_ids if tid is not None]
    m = len(tiles)
    SIZE = n * n

    start_pos = {tid: pos for pos, tid in enumerate(board_ids) if tid is not None}
    blank_start = board_ids.index(None)

    # Precompute allowed (pos, neighbour) transitions for blank
    allowed = [(p, nb)
               for p in range(SIZE)
               for nb in _neighbors(p, n)]

    model = cp_model.CpModel()

    # ── Variables ──────────────────────────────────────────────────────────────
    blank = [model.new_int_var(0, SIZE - 1, f"b{t}") for t in range(T_max + 1)]
    tp    = [[model.new_int_var(0, SIZE - 1, f"tp{t}_{i}") for i in range(m)]
             for t in range(T_max + 1)]

    # ── Boundary conditions ────────────────────────────────────────────────────
    model.add(blank[0] == blank_start)
    for i, tid in enumerate(tiles):
        model.add(tp[0][i] == start_pos[tid])
        model.add(tp[T_max][i] == assignment[tid])

    # ── Per-step constraints ───────────────────────────────────────────────────
    for t in range(T_max):
        # Blank moves to an adjacent cell
        model.add_allowed_assignments([blank[t], blank[t + 1]], allowed)

        # All positions distinct (tiles + blank)
        model.add_all_different([blank[t]] + [tp[t][i] for i in range(m)])

        # For each tile: moves iff it is currently at blank[t+1]
        for i in range(m):
            moved = model.new_bool_var(f"mv{t}_{i}")
            model.add(tp[t][i] == blank[t + 1]).only_enforce_if(moved)
            model.add(tp[t][i] != blank[t + 1]).only_enforce_if(~moved)
            model.add(tp[t + 1][i] == blank[t]).only_enforce_if(moved)
            model.add(tp[t + 1][i] == tp[t][i]).only_enforce_if(~moved)

    # Final step: all positions distinct
    model.add_all_different([blank[T_max]] + [tp[T_max][i] for i in range(m)])

    # ── Solve ─────────────────────────────────────────────────────────────────
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = timeout
    solver.parameters.num_workers = 8   # parallel search
    status = solver.solve(model)

    if status in (cp_model.OPTIMAL, cp_model.FEASIBLE):
        # Each move = position blank moved TO (= tile that slid into blank's old spot)
        return [solver.value(blank[t + 1]) for t in range(T_max)]

    return None


def _md_lc_admissible(board_ids: list, assignment: dict, n: int) -> int:
    """Admissible lower bound (MD+LC) — used to seed CP-SAT T search."""
    return _md_lc(tuple(board_ids), assignment, n)


# ── Main entry point ──────────────────────────────────────────────────────────

def compute_solution(board_ids: list, target_colors: list, n: int) -> list[int] | None:
    """
    Returns list of board position indices to move (same format as JS solver).

    Pipeline:
      All sizes:
        1. OR-Tools linear_sum_assignment → optimal tile-to-target mapping

      n=3 (3×3):
        2. A* + MD+LC → exact optimal in < 0.1s (no CP-SAT needed)

      n=4 (4×4):
        2. PDB A*     → fast upper bound T_ub (usually optimal)
        3. CP-SAT     → try to find solution with T_ub-1 moves (proves or improves)

      n=5 (5×5):
        2. PDB A*     → upper bound (much shorter than constructive)
        3. CP-SAT     → try to shorten further within time budget
        4. Constructive fallback if A* timed out
    """
    assignment = optimal_assignment(board_ids, target_colors, n)
    assignment = ensure_solvable_assignment(board_ids, assignment, target_colors, n)

    # n=3: A* with admissible MD+LC is already optimal
    if n == 3:
        return astar_solve(board_ids, assignment, n, timeout=10.0)

    # ── Fast upper bound ──────────────────────────────────────────────────────
    ub_result = astar_solve(board_ids, assignment, n, timeout=5.0)
    if ub_result is None:
        ub_result = constructive_solve(board_ids, assignment, n)
    T_ub = len(ub_result)

    # ── Admissible lower bound (MD+LC) ────────────────────────────────────────
    T_lb = _md_lc_admissible(board_ids, assignment, n)

    if T_lb >= T_ub:
        return ub_result   # already proven optimal

    # ── CP-SAT: try to find a shorter solution ────────────────────────────────
    # Budget: n=4 gets more time because state space is smaller
    import time
    cpsat_budget = {4: 20.0, 5: 25.0}
    deadline = time.time() + cpsat_budget[n]

    best = ub_result
    T_try = T_ub - 1

    while T_try >= T_lb and time.time() < deadline:
        remaining = deadline - time.time()
        if remaining < 1.0:
            break
        result = cpsat_solve(board_ids, assignment, n,
                             T_max=T_try,
                             timeout=remaining * 0.85)
        if result is not None:
            best = result
            T_try -= 1       # found it — try one shorter
        else:
            break            # UNSAT or timeout

    return best
