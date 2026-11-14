"""Reference car code for the 16x16 micromouse maze simulator.

Upload any .py file in the simulator's "Car code" tab. The file must define:

    class CarController:
        def __init__(self, car_index: int): ...
        def next_move(self, state: dict) -> str: ...

next_move(state) must return one of these action strings:

    "forward"   drive one cell ahead (a real wall blocks the move)
    "left"      rotate 90 degrees counter-clockwise (one step)
    "right"     rotate 90 degrees clockwise (one step)
    "back"      rotate 180 degrees (one step)
    "stop"      retire this car; its run ends

state dictionary keys:

    position   (x, y) current cell; x = column 0..15, y = row 0..15
    direction  0=N 1=E 2=S 3=W (same convention as the Arduino firmware)
    sensors    {"front": bool, "left": bool, "right": bool} wall indicators
    walls      learned wall map, walls[y][x][direction] -> True means a wall
               (walls only become known after the car has sensed the cell)
    visited    set of (x, y) cells this car has already occupied
    goal       the (x, y) centre cell assigned to this car
    quadrant   {"x": range, "y": range} the 8x8 area this car must stay in
    scenario   "explore" (sweep the quadrant first) or "shortest" (beeline)
    size       16

Each call to next_move is one simulation step. A car's run ends when it
reaches its goal cell or returns "stop".

This template implements both scenarios with flood-fill navigation: it
commits to a target (nearest unvisited cell while sweeping, or the centre
goal), then repeatedly steps to the neighbouring cell that is closest to
the target according to the best route it currently knows. Adapt it or
replace it with your own logic.
"""

from collections import deque

DX = (0, 1, 0, -1)
DY = (-1, 0, 1, 0)


class CarController:
    """Flood-fill style controller used as the reference implementation."""

    def __init__(self, car_index: int = 0) -> None:
        self.car_index = car_index
        self.target = None

    def _bfs(self, state: dict, root, allowed: set, avoid=frozenset()) -> dict:
        """Optimistic BFS distances from `root` over known-open walls
        (unsensed walls are assumed open), restricted to `allowed`.
        Cells in `avoid` are treated as blocked."""
        walls = state["walls"]
        dist = {root: 0}
        queue = deque([root])
        while queue:
            x, y = queue.popleft()
            for d in range(4):
                nx, ny = x + DX[d], y + DY[d]
                if (nx, ny) not in allowed or (nx, ny) in dist:
                    continue
                if (nx, ny) in avoid:
                    continue
                if walls[y][x][d]:
                    continue
                dist[(nx, ny)] = dist[(x, y)] + 1
                queue.append((nx, ny))
        return dist

    def _pick_target(self, state: dict, position, goal, allowed: set):
        """Choose the next committed target cell for this car."""
        visited = state["visited"]
        if state["scenario"] == "explore":
            unvisited = allowed - visited - {goal}
            if unvisited:
                dist = self._bfs(state, position, allowed, avoid={goal})
                candidates = [c for c in unvisited if c in dist]
                if candidates:
                    return min(candidates, key=lambda c: dist[c])
            return goal
        return goal

    def next_move(self, state: dict) -> str:
        position = state["position"]
        goal = tuple(state["goal"])
        quadrant = state["quadrant"]
        allowed = set((x, y) for x in quadrant["x"] for y in quadrant["y"])

        if self.target is None or self.target == position:
            self.target = self._pick_target(state, position, goal, allowed)
        if self.target is None or self.target == position:
            return "stop"

        dist = self._bfs(state, self.target, allowed)
        if position not in dist:
            # Learned walls say the target cannot be reached: re-plan.
            self.target = self._pick_target(state, position, goal, allowed)
            if self.target is None or self.target == position:
                return "stop"
            dist = self._bfs(state, self.target, allowed)
            if position not in dist:
                return "stop"

        here = dist[position]
        walls = state["walls"]
        best = None
        for d in range(4):
            if walls[position[1]][position[0]][d]:
                continue
            neighbour = (position[0] + DX[d], position[1] + DY[d])
            value = dist.get(neighbour)
            if value is not None and value < here:
                if best is None or value < best[0]:
                    best = (value, d)
        if best is None:
            return "stop"

        turn = (best[1] - state["direction"]) % 4
        if turn == 0:
            return "forward"
        if turn == 1:
            return "right"
        if turn == 2:
            return "back"
        return "left"
