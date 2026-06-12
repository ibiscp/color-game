from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from solver import compute_solution

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["POST"],
    allow_headers=["Content-Type"],
)


class SolveRequest(BaseModel):
    board: list[str | None]   # tile IDs, e.g. "red-0", or None for blank
    target: list[str | None]  # color strings, e.g. "red", or None for blank
    n: int


@app.post("/solve")
async def solve(req: SolveRequest):
    if len(req.board) != req.n * req.n or len(req.target) != req.n * req.n:
        raise HTTPException(status_code=400, detail="Board/target length must equal n*n")

    moves = compute_solution(req.board, req.target, req.n)
    if moves is None:
        raise HTTPException(status_code=422, detail="Solver could not find a solution in time")

    return {"moves": moves, "num_moves": len(moves)}
