"""Micromouse Car 2 controller - port of the car's real MicroPython strategy.

Source: "Micromouse Car 2 Codes (TQD JS2)/car2/micromouse.py" (mouseSearch, coordinateUpdate)
        + "car2/lib/maze.py" (centralMethod, crosswayCount, mapstepEdit,
        objectGoto)

The original algorithm (16x16 maze, walls learned from IR with symmetric
propagation to neighbouring cells):

1. Exploration = goal-biased DFS (mouseSearch, micromouse.py:1010):
   - crosswayCount() counts open directions whose neighbouring cell is not
     yet explored (maze.py:154).
   - count > 1  -> push this cell on the crossroad stack.
   - crosswayChoice() / centralMethod() (maze.py:516) picks the next
     direction: a fixed priority per goal-relative region, always biased
     toward the goal and restricted to unexplored cells.
   - count == 0 -> mapstepEdit() floods step-distances from the target;
     objectGoto() follows the descending gradient to the goal, or backtracks
     to the most recent crossroad on the stack.
   - Goal cells of OTHER cars are pre-marked explored (cooperateInit /
     cooperateXxx, micromouse.py:131/1106) so each car sweeps only its own
     area first, then drives to its own centre goal.
2. RacingRun() then drives the shortest known route through the learned map
   (objectGoto on the flood map).

This port uses the simulator's direction convention (0=N, 1=E, 2=S, 3=W)
and reproduces the same decision flow. Scenario mapping:
  - "explore"  -> mouseSearch biased DFS: sweep every reachable cell of the
                  quadrant (goal treated as pre-explored, like the car's
                  cooperateInit marking), then flood-gradient to the goal.
  - "shortest" -> objectGoto straight to the goal (flood-gradient descent on
                  the optimistically-flooded learned map).
"""

from collections import deque

DX = (0, 1, 0, -1)
DY = (-1, 0, 1, 0)
SIZE = 16

# Cells held back for the final approach, mirroring the built-in rule: the
# four cells leading into each car's centre goal are entered only after
# every other cell of the quadrant has been swept - and the car walks to
# the far end (the flood-fill 4) first, then descends to the goal.
RESERVED_CORRIDOR = {
    0: frozenset({(7, 3), (7, 4), (7, 5), (7, 6)}),
    1: frozenset({(8, 3), (8, 4), (8, 5), (8, 6)}),
    2: frozenset({(7, 9), (7, 10), (7, 11), (7, 12)}),
    3: frozenset({(8, 9), (8, 10), (8, 11), (8, 12)}),
}


def _opposite(d):
    return (d + 2) % 4


class CarController:
    """Goal-biased DFS explorer with crossroad backtracking, as mouseSearch."""

    def __init__(self, car_index: int = 0) -> None:
        self.car_index = car_index

    # ---- maze primitives (maze.py equivalents) ---------------------------

    def _crossway_dirs(self, state, position, allowed, include_goal):
        """crosswayCount(): open directions whose neighbour is unexplored."""
        walls = state["walls"]
        visited = state["visited"]
        result = []
        for d in range(4):
            if walls[position[1]][position[0]][d]:
                continue
            neighbour = (position[0] + DX[d], position[1] + DY[d])
            if neighbour not in allowed:
                continue
            if neighbour in visited:
                continue
            if not include_goal and neighbour == tuple(state["goal"]):
                continue  # goal pre-marked as explored (cooperateInit)
            result.append(d)
        return result

    def _priority_order(self, position, goal):
        """centralMethod(): direction order biased toward the goal."""
        dx = goal[0] - position[0]
        dy = goal[1] - position[1]
        horizontal = 1 if dx > 0 else 3
        vertical = 2 if dy > 0 else 0
        if abs(dx) >= abs(dy):
            order = (
                horizontal, vertical,
                _opposite(vertical), _opposite(horizontal),
            )
        else:
            order = (
                vertical, horizontal,
                _opposite(horizontal), _opposite(vertical),
            )
        return order

    def _flood(self, state, target, allowed, avoid=frozenset()):
        """mapstepEdit(): optimistic flood of step distances from target.
        Unknown walls count as open (the car has not measured them yet).
        Cells in `avoid` are excluded from all routes (e.g. the centre goal
        during exploration, so backtracking never ends the run early)."""
        walls = state["walls"]
        target = tuple(target)
        if target in avoid:
            return {}
        dist = {target: 0}
        queue = deque([target])
        while queue:
            x, y = queue.popleft()
            for d in range(4):
                nx, ny = x + DX[d], y + DY[d]
                neighbour = (nx, ny)
                if neighbour not in allowed or neighbour in dist:
                    continue
                if neighbour in avoid or walls[y][x][d]:
                    continue
                dist[neighbour] = dist[(x, y)] + 1
                queue.append(neighbour)
        return dist

    def _gradient_step(self, state, dist, position):
        """objectGoto(): one step down the distance gradient.

        Directions are tried in the car's own priority order - forward,
        left, right, back - so the car keeps driving straight whenever the
        gradient allows it, exactly like the built-in scenario, instead of
        turning at every junction where a side direction also descends."""
        here = dist[position]
        walls = state["walls"]
        best = None
        for relative in (0, 3, 1, 2):
            d = (state["direction"] + relative) % 4
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

    # ---- main decision (mouseSearch) -------------------------------------

    def next_move(self, state: dict) -> str:
        position = state["position"]
        goal = tuple(state["goal"])
        quadrant = state["quadrant"]
        allowed = set((x, y) for x in quadrant["x"] for y in quadrant["y"])

        if state["scenario"] != "explore":
            # RacingRun speed phase: objectGoto straight to the goal.
            if position == goal:
                return "stop"
            dist = self._flood(state, goal, allowed)
            if position not in dist:
                return "stop"
            return self._gradient_step(state, dist, position)

        # --- exploration phase --------------------------------------------
        reserved = RESERVED_CORRIDOR.get(self.car_index, frozenset())
        open_dirs = self._crossway_dirs(
            state, position, allowed, include_goal=False
        )
        # The reserved corridor stays closed until everything else in the
        # quadrant has been swept (same rule as the built-in sweep).
        open_dirs = [
            d for d in open_dirs
            if (position[0] + DX[d], position[1] + DY[d]) not in reserved
        ]
        if open_dirs:
            if state.get("sweep_order", "far") == "far":
                # Clear the cells farthest from the goal first, so the goal
                # area and the final goal run come last. Ties prefer the
                # car's own direction order (forward, left, right, back),
                # matching the built-in sweep so the car does not turn
                # away from a corridor it could simply continue down.
                dist_goal = self._flood(state, goal, allowed)
                best = None
                for relative in (0, 3, 1, 2):
                    d = (state["direction"] + relative) % 4
                    if d not in open_dirs:
                        continue
                    neighbour = (position[0] + DX[d], position[1] + DY[d])
                    value = dist_goal.get(neighbour, 10 ** 6)
                    if best is None or value > best[0]:
                        best = (value, d)
                return self._turn_then(best[1], state)
            order = self._priority_order(position, goal)
            for d in order:
                if d in open_dirs:
                    return self._turn_then(d, state)
            return self._turn_then(open_dirs[0], state)

        # No unexplored direction here: walk to the cell that still has an
        # unexplored exit, so every reachable cell gets swept before the
        # goal run (the original mouseSearch abandons the sweep as soon as
        # the goal is reachable; this adapter completes it instead).
        def frontier_from(avoid):
            dist_here = self._flood(state, position, allowed, avoid=avoid)
            frontier = []
            walls = state["walls"]
            visited = state["visited"]
            for cell, value in dist_here.items():
                for d in range(4):
                    if walls[cell[1]][cell[0]][d]:
                        continue
                    neighbour = (cell[0] + DX[d], cell[1] + DY[d])
                    if (
                        neighbour in allowed
                        and neighbour not in visited
                        and neighbour != goal
                        and neighbour not in reserved
                    ):
                        frontier.append((value, cell))
                        break
            return frontier

        frontier = frontier_from(reserved | {goal})
        if not frontier:
            # Nothing left outside the corridor. If some cells can only be
            # reached through it, sweep them first; otherwise the corridor
            # finale below takes over.
            leftover = allowed - state["visited"] - reserved - {goal}
            if leftover:
                frontier = frontier_from({goal})
        if frontier:
            # Same target choice as the built-in sweep: the frontier cell
            # FARTHEST from the goal first, nearest walk distance on ties.
            dist_goal = self._flood(state, goal, allowed)
            target = min(
                frontier,
                key=lambda item: (-dist_goal.get(item[1], 0), item[0]),
            )[1]
            dist_target = self._flood(
                state, target, allowed, avoid=reserved | {goal}
            )
            if position not in dist_target:
                dist_target = self._flood(state, target, allowed, avoid={goal})
            if position not in dist_target:
                return "stop"
            return self._gradient_step(state, dist_target, position)

        # Corridor finale: walk to the far end of the reserved corridor
        # (the flood-fill 4) first, then the final goal run below descends
        # 4 -> 3 -> 2 -> 1 -> goal, matching the built-in scenario.
        unvisited_reserved = [c for c in reserved if c not in state["visited"]]
        if unvisited_reserved:
            dist_goal = self._flood(state, goal, allowed)
            corridor_target = max(
                unvisited_reserved,
                key=lambda cell: dist_goal.get(cell, 0),
            )
            dist_corr = self._flood(
                state, corridor_target, allowed, avoid={goal}
            )
            if position not in dist_corr:
                # A pocket reachable only through the goal: fall back to
                # the open map (crossing the goal mid-sweep is safe - the
                # run only ends once the corridor is done and the car is
                # back on the goal).
                dist_corr = self._flood(state, corridor_target, allowed)
            if position in dist_corr:
                return self._gradient_step(state, dist_corr, position)

        # Sweep complete: drive to the centre goal (and stop on arrival).
        if position == goal:
            return "stop"
        dist = self._flood(state, goal, allowed)
        if position not in dist:
            return "stop"
        return self._gradient_step(state, dist, position)

    def _turn_then(self, d, state):
        turn = (d - state["direction"]) % 4
        if turn == 0:
            return "forward"
        if turn == 1:
            return "right"
        if turn == 2:
            return "back"
        return "left"
