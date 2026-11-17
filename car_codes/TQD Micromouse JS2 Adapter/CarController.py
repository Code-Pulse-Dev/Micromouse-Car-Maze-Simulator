"""Micromouse Car 2 controller - port of the car's real MicroPython strategy.

Source: "Micromouse Car 2 Codes (TQD JS2)/car2/micromouse.py" (mouseSearch, coordinateUpdate)
        + "car2/lib/maze.py" (centralMethod, crosswayCount, mapstepEdit,
        objectGoto)

The original algorithm (16x16 maze, walls learned from IR with symmetric
propagation to neighbouring cells):

1. Exploration = goal-biased sweep (mouseSearch, micromouse.py:1010):
   the car clears every reachable cell of its quadrant before the goal
   run, keeping wall memory so nothing is revisited.
2. RacingRun() then drives the shortest known route through the learned
   map (objectGoto on the flood map).

IMPORTANT implementation note: the web simulator passes the state over as
JavaScript structures (arrays / proxy objects). Every coordinate is
therefore normalised to plain Python ints at the top of next_move();
otherwise membership tests like `neighbour in visited` silently fail
(a tuple never matches a JS array element), the car believes nothing has
been explored, re-plans every tick and spins in place. The desktop app
passes native Python types, which these conversions leave untouched.

Scenario mapping:
  - "explore"  -> quadrant sweep (built-in clearing order), then
                  objectGoto flood-gradient to the goal.
  - "shortest" -> objectGoto straight to the goal (flood-gradient descent
                  on the optimistically-flooded learned map).
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


class CarController:
    """Quadrant sweeper with wall memory, objectGoto goal run."""

    def __init__(self, car_index: int = 0) -> None:
        self.car_index = car_index

    # ---- maze primitives (maze.py equivalents) ---------------------------

    def _open_dirs(self, state, position, qx, qy):
        """Open directions whose neighbour is inside the quadrant.
        Unknown walls count as open (not sensed yet)."""
        walls = state["walls"]
        result = []
        for d in range(4):
            if walls[position[1]][position[0]][d]:
                continue
            nx, ny = position[0] + DX[d], position[1] + DY[d]
            if nx not in qx or ny not in qy:
                continue
            result.append(d)
        return result

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
        gradient allows it, exactly like the built-in scenario."""
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

    def _turn_then(self, d, state):
        turn = (d - state["direction"]) % 4
        if turn == 0:
            return "forward"
        if turn == 1:
            return "right"
        if turn == 2:
            return "back"
        return "left"

    # ---- main decision ----------------------------------------------------

    def next_move(self, state: dict) -> str:
        # Normalise the incoming state (see the module docstring): the web
        # hands over JS arrays, so coordinates become plain Python ints -
        # without this the visited checks fail and the car spins.
        position = (int(state["position"][0]), int(state["position"][1]))
        goal = (int(state["goal"][0]), int(state["goal"][1]))
        visited = {(int(v[0]), int(v[1])) for v in state["visited"]}
        qx = {int(v) for v in state["quadrant"]["x"]}
        qy = {int(v) for v in state["quadrant"]["y"]}
        allowed = set((x, y) for x in qx for y in qy)

        if state["scenario"] != "explore":
            # RacingRun speed phase: objectGoto straight to the goal.
            if position == goal:
                return "stop"
            dist = self._flood(state, goal, allowed)
            if position not in dist:
                return "stop"
            return self._gradient_step(state, dist, position)

        # --- exploration phase --------------------------------------------
        # Same clearing pattern as the built-in scenario: candidate
        # directions forward, left, right only (a U-turn is never a
        # candidate - the frontier walk below handles going back), the
        # cells farthest from the goal first, and the held-back corridor
        # entered last.
        reserved = RESERVED_CORRIDOR.get(self.car_index, frozenset())
        front = state["direction"]
        left = (front + 3) % 4
        right = (front + 1) % 4
        open_dirs = set(self._open_dirs(state, position, qx, qy))

        # Sweep regions: car 2 clears the top half of its quadrant
        # COMPLETELY before it starts on the bottom half (same rule as the
        # built-in sweep). All other cars sweep the whole quadrant at once.
        regions = [allowed]
        if self.car_index == 1:
            mid = (min(qy) + max(qy) + 1) // 2
            top = {cell for cell in allowed if cell[1] < mid}
            top_unvisited = [
                cell for cell in top
                if cell not in visited
                and cell != goal
                and cell not in reserved
            ]
            borrowed = bool(getattr(self, "_phase_borrowed", False))
            if position[1] < mid:
                borrowed = False
                self._phase_borrowed = False
            if top_unvisited and (borrowed or position[1] < mid):
                # The phase only counts while the car is still up there
                # (or is climbing back from a borrowed cell) and a top-half
                # cell is reachable without crossing into the bottom half.
                if borrowed:
                    regions.insert(0, top)
                else:
                    reach = self._flood(state, position, top, avoid=reserved)
                    if any(cell in reach for cell in top_unvisited):
                        regions.insert(0, top)

        for region in regions:
            # 1. Take a direction that leads somewhere new (never back).
            candidates = []
            for d in (front, left, right):
                if d not in open_dirs:
                    continue
                neighbour = (position[0] + DX[d], position[1] + DY[d])
                if (
                    neighbour in visited
                    or neighbour == goal
                    or neighbour in reserved
                    or neighbour not in region
                ):
                    continue
                candidates.append((d, neighbour))
            if candidates:
                if state.get("sweep_order", "far") == "far":
                    dist_goal = self._flood(state, goal, allowed)
                    # Same order as the built-in sweep: straight ahead
                    # first, then farther from the goal, then the car's
                    # own direction order.
                    candidates.sort(
                        key=lambda item: (
                            (item[0] - front) % 4 != 0,
                            -dist_goal.get(item[1], SIZE * SIZE),
                            (item[0] - front) % 4,
                        )
                    )
                return self._turn_then(candidates[0][0], state)

            if region is not allowed and not borrowed:
                # Straight continuation over the phase boundary (same rule
                # as the built-in sweep): take the cell ahead now, then
                # climb back up for the rest of the top half.
                ahead = (position[0] + DX[front], position[1] + DY[front])
                if (
                    front in open_dirs
                    and ahead not in visited
                    and ahead != goal
                    and ahead not in reserved
                    and ahead in allowed
                ):
                    self._phase_borrowed = True
                    return "forward"

            # 2. Dead end / swept area: walk to the cell that still has an
            #    unvisited exit, so every reachable cell gets swept before
            #    the goal run. Routes stay out of the reserved corridor and
            #    never pass through the centre goal.
            def frontier_from(avoid, region=region, area=None):
                dist_here = self._flood(state, position, area or region, avoid=avoid)
                frontier = []
                walls = state["walls"]
                for cell, value in dist_here.items():
                    for d in range(4):
                        if walls[cell[1]][cell[0]][d]:
                            continue
                        neighbour = (cell[0] + DX[d], cell[1] + DY[d])
                        if (
                            neighbour in region
                            and neighbour not in visited
                            and neighbour != goal
                            and neighbour not in reserved
                        ):
                            frontier.append((value, cell))
                            break
                return frontier

            frontier = frontier_from(reserved | {goal}) if reserved else []
            if not frontier and region is not allowed:
                if borrowed:
                    # Climb back out of the borrowed cell on the open map,
                    # still aiming at the top half.
                    frontier = frontier_from(reserved, area=allowed)
                if not frontier:
                    # Top half exhausted (any pockets through the bottom
                    # half are picked up in the full-quadrant pass).
                    continue
            if not frontier:
                # Nothing left outside the corridor. If some cells can only
                # be reached through it, sweep them first; otherwise the
                # corridor finale below takes over.
                leftover = allowed - visited - reserved - {goal}
                if leftover:
                    frontier = frontier_from({goal})
            if frontier:
                # Same target choice as the built-in sweep: the frontier
                # cell FARTHEST from the goal first, nearest walk distance
                # on ties.
                dist_goal = self._flood(state, goal, allowed)
                target = min(
                    frontier,
                    key=lambda item: (-dist_goal.get(item[1], 0), item[0]),
                )[1]
                dist_target = self._flood(
                    state, target, region, avoid=reserved | {goal}
                )
                if position not in dist_target:
                    dist_target = self._flood(state, target, allowed)
                if position not in dist_target:
                    return "stop"
                return self._gradient_step(state, dist_target, position)

        # 2b. Corridor finale: walk to the far end of the reserved corridor
        #     (the flood-fill 4) first, then the final goal run below
        #     descends 4 -> 3 -> 2 -> 1 -> goal, matching the built-in.
        unvisited_reserved = [c for c in reserved if c not in visited]
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

        # 3. Sweep complete: objectGoto to the centre goal (and stop).
        if position == goal:
            return "stop"
        dist = self._flood(state, goal, allowed)
        if position not in dist:
            return "stop"
        return self._gradient_step(state, dist, position)
