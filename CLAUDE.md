# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

A browser-based sliding-tile color puzzle game (single `index.html`) paired with a Python solver service (`solver/`) that finds optimal move sequences via OR-Tools.

## Solver service

All Python work lives in `solver/`. Dependencies are managed with **uv**.

```bash
cd solver

# Install dependencies (first time)
uv venv && uv pip install -r requirements.txt

# Precompute PDB tables — required once before solving 4×4 or 5×5 (takes ~4–5 min, parallelised)
uv run python pdb.py --n 4 5

# Start the API server
uv run uvicorn app:app --port 8000
```

The server exposes one endpoint: `POST /solve` — see `app.py` for the request/response schema.

## Architecture

### Game (`index.html`)
Single self-contained file. Key JS sections (all inside one IIFE):
- **State**: `board[]` (tile objects `{id, color}` or `null`), `target[]` (color strings or `null`), `N` (board dimension 3/4/5).
- **Solver integration**: `computeSolution()` (async) calls `http://localhost:8000/solve` first; falls back to the built-in JS solver if the server is unreachable.
- **Built-in JS solvers**: BFS for 3×3 (`bfsSolve`), constructive row-peeling for 4×4/5×5 (`constructiveSolve`).
- **Auto-play**: `startAuto()` executes the returned move list with `playerMove()` at adaptive speed.

### Solver pipeline (`solver/`)

`compute_solution()` in `solver.py` is the main entry point:

1. **OR-Tools `linear_sum_assignment`** (`optimal_assignment`) — finds the minimum-Manhattan-distance mapping from tile IDs to target positions, per color group. Parity is checked and fixed if needed (`ensure_solvable_assignment`).

2. **A\* with PDB heuristic** (`astar_solve`) — for n=3 uses Manhattan+Linear-Conflict (admissible); for n≥4 uses the additive disjoint Pattern Database loaded from `pdb_cache/`.

3. **OR-Tools CP-SAT** (`cpsat_solve`) — for n≥4, tries to find a solution with one fewer move than A\*, proving optimality or improving the result. Skipped for n=3 (A\* is already optimal via admissible heuristic).

4. **Constructive fallback** (`constructive_solve`) — row-by-row BFS placement; always terminates but produces longer solutions. Only reached if A\* times out.

### PDB (`pdb.py`)
Pattern Databases are precomputed per `(n, blank_target)` pair and cached as pickle files in `solver/pdb_cache/`. Each file covers one disjoint group of tiles. On first use for an unseen `blank_target`, the table is built automatically (~14 s per group) and saved. `precompute_all()` builds all combinations in parallel using `ProcessPoolExecutor`.

Group sizes: 5 tiles/group for n=4 (3 groups), 4 tiles/group for n=5 (6 groups).

## Move format

Both the JS solver and the Python API return moves as a list of **board position indices** — each entry is the index of the tile that slides into the blank. This is identical to what `playerMove(idx)` in `index.html` expects.

## Data flow: tile IDs vs colors

- `board` holds tile objects with IDs like `"red-0"`, `"red-1"` (distinguishes same-color tiles).
- `target` holds plain color strings like `"red"` (any red tile satisfies any red target cell).
- The solver receives both: IDs allow OR-Tools to find the optimal same-color assignment; colors define the goal.
