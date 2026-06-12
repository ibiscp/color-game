"""
Additive Disjoint Pattern Database (PDB) heuristic for the N-puzzle.

Design
──────
After OR-Tools assignment each tile has a fixed target position.
One grid cell is the blank's target (blank_target).

Groups partition the n²-1 non-blank target positions.
For each group we run BFS backward from the goal, tracking only that group's
tile positions (non-group tiles are wildcards).  The stored value is the
minimum number of moves to place those tiles, regardless of where the blank
or other tiles are.

Summing the PDB values across all disjoint groups is admissible (each move
can only help one group at a time) and usually far tighter than MD+LC.

Cache
─────
Tables are pickled to pdb_cache/pdb_n{N}_bt{blank_target}_g{g}.pkl.
Key: (n, blank_target) — each blank target position gets its own set of tables.
For n=4: 16 possible blank targets × 3 groups each.
For n=5: 25 possible blank targets × 6 groups each.

First call for a given (n, blank_target) builds and saves; later calls load.
"""

import os
import pickle
import time
from collections import deque
from concurrent.futures import ProcessPoolExecutor, as_completed


# ── Grid helpers ───────────────────────────────────────────────────────────────

def _grid_nbrs(n: int, i: int) -> list[int]:
    r, c = divmod(i, n)
    out = []
    if r > 0:    out.append(i - n)
    if r < n-1:  out.append(i + n)
    if c > 0:    out.append(i - 1)
    if c < n-1:  out.append(i + 1)
    return out


# ── PDB computation ────────────────────────────────────────────────────────────

def build_pdb_for_group(n: int, group_goals: list[int]) -> dict[tuple, int]:
    """
    BFS backward from goal. Non-group tiles are wildcards.
    Returns {tuple_of_current_group_positions: min_moves_to_goal}.
    """
    nbrs = [_grid_nbrs(n, i) for i in range(n * n)]
    ggoals_t = tuple(group_goals)
    occupied = frozenset(ggoals_t)

    pdb: dict[tuple, int] = {ggoals_t: 0}
    visited: dict[tuple, int] = {}
    queue: deque = deque()

    for b in range(n * n):
        if b not in occupied:
            state = (b, ggoals_t)
            visited[state] = 0
            queue.append(state)

    while queue:
        b, gpos = queue.popleft()
        dist = visited[(b, gpos)]
        gset = frozenset(gpos)

        for nb in nbrs[b]:
            if nb in gset:
                idx = gpos.index(nb)
                ng = list(gpos)
                ng[idx] = b
                ng_t = tuple(ng)
                nb2 = nb
            else:
                ng_t = gpos
                nb2 = nb

            nstate = (nb2, ng_t)
            if nstate not in visited:
                ndist = dist + 1
                visited[nstate] = ndist
                queue.append(nstate)
                if ng_t not in pdb or pdb[ng_t] > ndist:
                    pdb[ng_t] = ndist

    return pdb


def _build_and_save(args):
    """Worker function for parallel precomputation."""
    n, group_goals, path = args
    if os.path.exists(path):
        return path, True   # already cached
    pdb = build_pdb_for_group(n, group_goals)
    with open(path, "wb") as f:
        pickle.dump(pdb, f)
    return path, False


# ── PatternDatabase class ──────────────────────────────────────────────────────

# Group sizes per board size
_GROUP_SIZES = {4: 5, 5: 4}


class PatternDatabase:
    """
    Additive disjoint PDB for one (n, blank_target) combination.

    blank_target  =  the board position where the blank should end up.
                     This is the only position NOT covered by any group.
    """

    def __init__(self, n: int, blank_target: int, cache_dir: str = "pdb_cache"):
        self.n = n
        self.blank_target = blank_target
        k = _GROUP_SIZES.get(n, 0)
        if k == 0:
            self.groups: list[list[int]] = []
            self.tables: list[dict] = []
            self._ready = False
            return

        # All non-blank target positions, split into groups of k
        sorted_targets = sorted(set(range(n * n)) - {blank_target})
        self.groups = [sorted_targets[i: i + k] for i in range(0, len(sorted_targets), k)]

        os.makedirs(cache_dir, exist_ok=True)
        self.tables = []
        self._ready = False

        for gi, group in enumerate(self.groups):
            path = os.path.join(cache_dir, f"pdb_n{n}_bt{blank_target}_g{gi}.pkl")
            if os.path.exists(path):
                with open(path, "rb") as f:
                    self.tables.append(pickle.load(f))
            else:
                print(f"[PDB] building n={n} bt={blank_target} group {gi} "
                      f"(targets {group[0]}–{group[-1]})…", flush=True)
                t0 = time.time()
                tbl = build_pdb_for_group(n, group)
                print(f"[PDB]   done: {len(tbl):,} entries in {time.time()-t0:.1f}s")
                with open(path, "wb") as f:
                    pickle.dump(tbl, f)
                self.tables.append(tbl)

        self._ready = True

    @property
    def ready(self) -> bool:
        return self._ready

    def heuristic(self, board_ids: tuple, assignment: dict) -> int:
        """
        Admissible heuristic value for the current board.

        board_ids   : tuple of tile IDs (or None for blank) at each position.
        assignment  : {tile_id: target_position} — fixed for this puzzle.

        Returns sum of PDB lookups across all groups.
        """
        # Build: target_position → current_position
        target_to_current: dict[int, int] = {}
        for pos, tid in enumerate(board_ids):
            if tid is not None:
                target_to_current[assignment[tid]] = pos

        total = 0
        for group_goals, table in zip(self.groups, self.tables):
            gpos = tuple(target_to_current[g] for g in group_goals)
            total += table.get(gpos, 0)
        return total


# ── PDB cache ──────────────────────────────────────────────────────────────────

_pdb_cache: dict[tuple, PatternDatabase] = {}


def get_pdb(n: int, blank_target: int, cache_dir: str = "pdb_cache") -> PatternDatabase:
    key = (n, blank_target)
    if key not in _pdb_cache:
        _pdb_cache[key] = PatternDatabase(n, blank_target, cache_dir)
    return _pdb_cache[key]


def precompute_all(n: int, cache_dir: str = "pdb_cache",
                   max_workers: int | None = None) -> None:
    """
    Precompute and cache PDB tables for ALL blank_target values of board size n.
    Uses a ProcessPoolExecutor so groups run in parallel.
    """
    import multiprocessing
    os.makedirs(cache_dir, exist_ok=True)
    k = _GROUP_SIZES.get(n, 0)
    if k == 0:
        return

    # Collect all (blank_target, group_index, group_goals) combos
    tasks = []
    for bt in range(n * n):
        sorted_targets = sorted(set(range(n * n)) - {bt})
        groups = [sorted_targets[i: i + k] for i in range(0, len(sorted_targets), k)]
        for gi, group in enumerate(groups):
            path = os.path.join(cache_dir, f"pdb_n{n}_bt{bt}_g{gi}.pkl")
            tasks.append((n, group, path))

    total = len(tasks)
    workers = max_workers or min(multiprocessing.cpu_count(), total)
    print(f"[PDB] precomputing n={n}: {total} groups, {workers} workers", flush=True)
    t0 = time.time()

    done = 0
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futs = {pool.submit(_build_and_save, t): t for t in tasks}
        for fut in as_completed(futs):
            path, was_cached = fut.result()
            done += 1
            status = "cached" if was_cached else "built"
            print(f"[PDB]  ({done}/{total}) {status}: {os.path.basename(path)}",
                  flush=True)

    print(f"[PDB] n={n} done in {time.time()-t0:.1f}s", flush=True)


# ── CLI: precompute all blank_target variants ──────────────────────────────────

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Precompute PDB tables")
    parser.add_argument("--n", type=int, nargs="+", default=[4, 5])
    parser.add_argument("--cache-dir", default="pdb_cache")
    parser.add_argument("--workers", type=int, default=None,
                        help="parallel workers (default: cpu_count)")
    args = parser.parse_args()

    for n in args.n:
        precompute_all(n, cache_dir=args.cache_dir, max_workers=args.workers)

    print("\n✓ All done.")
