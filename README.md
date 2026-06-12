# Jogo das Cores

A browser-based sliding-tile puzzle game where you arrange colored tiles to match a target configuration.

**[▶ Play online](https://ibiscp.github.io/color-game/)**

![Jogo das Cores](https://ibiscp.github.io/color-game/preview.png)

## How to play

1. Choose a difficulty — **Fácil** (3×3), **Médio** (4×4), or **Difícil** (5×5)
2. The left panel shows the **target** configuration; the right panel is your **board**
3. Slide tiles into the blank space until the board matches the target
4. Click **🤖 Resolver** to watch the optimal solver do it automatically

Controls: click/tap a tile adjacent to the blank, drag tiles, or use arrow keys.

## Optimal solver

The **Resolver** button calls a local Python service that finds provably optimal solutions:

| Size | Algorithm | Typical moves | Time |
|------|-----------|--------------|------|
| 3×3 | A\* + Manhattan/Linear-Conflict | optimal | < 0.1 s |
| 4×4 | A\* + Pattern Database + CP-SAT | optimal | < 0.4 s |
| 5×5 | A\* + Pattern Database + CP-SAT | optimal | < 3 s |

### Running the solver service

```bash
cd solver

# First-time setup
uv venv && uv pip install -r requirements.txt

# Precompute Pattern Database tables (~4–5 min, runs once)
uv run python pdb.py --n 4 5

# Start the API server
uv run uvicorn app:app --port 8000
```

The browser falls back to the built-in JavaScript solver if the service is not running.

### How it works

1. **OR-Tools `linear_sum_assignment`** — finds the optimal assignment of same-color tiles to target positions (minimises total Manhattan distance)
2. **A\* + additive disjoint Pattern Databases** — searches for the shortest move sequence using precomputed lower bounds
3. **OR-Tools CP-SAT** — verifies optimality by proving no shorter solution exists

## Tech

- Frontend: vanilla HTML/CSS/JS, single file
- Solver: Python, [OR-Tools](https://developers.google.com/optimization), FastAPI
