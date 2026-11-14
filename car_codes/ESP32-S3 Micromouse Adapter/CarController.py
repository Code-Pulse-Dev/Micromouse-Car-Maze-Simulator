"""Micromouse Car 1 controller - port of the car's firmware logic, corrected.

Source: "Micromouse Car 1 Codes (ESP32-S3)/main/distance.cpp" -> navigateMaze()

The firmware's raw rule is:

    front wall -> turn left if open, else right if open, else U-turn
    otherwise  -> drive straight

Run as-is that rule loops forever between two cells around every dead end,
because the firmware keeps no map: after the U-turn the front is open
again and it drives back into the dead end it just left. This adapter
keeps the same decision style (front -> left -> right priority) but adds
what the firmware is still missing:

1. Wall memory  — the learned wall map (sensors + symmetry) is used, and
   the car never re-enters a visited cell while an unvisited one is open,
   so dead ends are left for good.
2. Backtracking — when the local cell has no unvisited exits, it walks
   (shortest known route) back to the nearest cell that still has one.
3. Goal logic   — once everything reachable in its quadrant is explored,
   it drives to its centre goal, which ends its run.

Porting note: to fix the real car, main.ino needs the same three things
(persist the wall bits per cell, backtrack to unexplored junctions, detect
the centre). The simulator adapter shows the intended behaviour.
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
    """Front-priority explorer with wall memory and backtracking."""

    def __init__(self, car_index: int = 0) -> None:
        self.car_index = car_index

    # ---- helpers ----------------------------------------------------------

    def _open_dirs(self, state, position):
        """Directions with no known wall, staying inside the quadrant.
        Unknown walls count as open (not sensed yet)."""
        walls = state["walls"]
        qx, qy = state["quadrant"]["x"], state["quadrant"]["y"]
        result = []
        for d in range(4):
            if walls[position[1]][position[0]][d]:
                continue
            nx, ny = position[0] + DX[d], position[1] + DY[d]
            if nx not in qx or ny not in qy:
                continue
            result.append(d)
        return result

    def _bfs(self, state, root, allowed, avoid=frozenset()):
        """Shortest-path distances from root over known-open edges.
        Cells in `avoid` are excluded from all routes (e.g. the centre goal
        during exploration, so backtracking never ends the run early)."""
        walls = state["walls"]
        root = tuple(root)
        if root in avoid:
            return {}
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

    def _step_towards(self, state, dist, position, target):
        """One descending step along the BFS gradient toward target.

        Tie-break matches the built-in sweep walker: directions are tried
        in the firmware priority order forward, left, right and only a
        strictly better distance replaces the current choice."""
        here = dist.get(position)
        if here is None or here == 0:
            return "stop"
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

    # ---- main decision ----------------------------------------------------

    def next_move(self, state: dict) -> str:
        position = state["position"]
        goal = tuple(state["goal"])
        quadrant = state["quadrant"]
        allowed = set(
            (x, y) for x in quadrant["x"] for y in quadrant["y"]
        )
        visited = state["visited"]

        # Speed-run scenario: the maze is treated as already known, so the
        # car drives the shortest route straight to its centre goal.
        if state["scenario"] == "shortest":
            if position == goal:
                return "stop"
            dist_goal = self._bfs(state, goal, allowed)
            if position not in dist_goal:
                return "stop"
            return self._step_towards(state, dist_goal, position, goal)

        # navigateMaze() priority: front -> left -> right (back = last resort)
        front = state["direction"]
        left = (front + 3) % 4
        right = (front + 1) % 4
        open_dirs = set(self._open_dirs(state, position))
        reserved = RESERVED_CORRIDOR.get(self.car_index, frozenset())

        # Sweep regions: car 2 clears the top half of its quadrant
        # COMPLETELY before it starts on the bottom half (same rule as the
        # built-in sweep). All other cars sweep the whole quadrant at once.
        regions = [allowed]
        if self.car_index == 1:
            zone_y = state["quadrant"]["y"]
            mid = (min(zone_y) + max(zone_y) + 1) // 2
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
                    reach = self._bfs(state, position, top, avoid=reserved)
                    if any(cell in reach for cell in top_unvisited):
                        regions.insert(0, top)

        for region in regions:
            # 1. Take a direction that leads somewhere new. Order: by
            #    default the FARTHEST cells from the goal are cleared
            #    first, so the goal area (and the final goal run) comes
            #    last. The reserved corridor is never entered while
            #    anything else is unexplored.
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
                    dist_goal = self._bfs(state, goal, allowed)
                    # Same order as the built-in sweep: straight ahead
                    # first (finish the cell in front instead of turning
                    # away and coming back for it), then farther from the
                    # goal, then forward < right < left < back.
                    candidates.sort(
                        key=lambda item: (
                            (item[0] - front) % 4 != 0,
                            -dist_goal.get(item[1], SIZE * SIZE),
                            (item[0] - front) % 4,
                        )
                    )
                d = candidates[0][0]
                turn = (d - front) % 4
                if turn == 0:
                    return "forward"
                if turn == 1:
                    return "right"
                return "left"

            if region is not allowed and not borrowed:
                # Straight continuation over the phase boundary: when the
                # corridor the car is already driving in carries on into
                # the other half, take that cell now instead of walking
                # all the way back for it later (same rule as the built-in
                # sweep). One borrowed cell, then it climbs back up.
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

            # 2. Dead end / swept area: walk to the nearest cell that still
            #    has an unvisited exit (corrects the firmware's U-turn
            #    loop). Routes stay out of the reserved corridor and never
            #    pass through the centre goal, so the sweep is not cut
            #    short.
            def frontier_from(avoid, region=region, area=None):
                dist = self._bfs(state, position, area or region, avoid=avoid)
                found = []
                for cell, value in dist.items():
                    for d in self._open_dirs(state, cell):
                        neighbour = (cell[0] + DX[d], cell[1] + DY[d])
                        if (
                            neighbour not in visited
                            and neighbour != goal
                            and neighbour not in reserved
                            and neighbour in region
                        ):
                            found.append((value, cell))
                            break
                return found

            frontier = frontier_from(reserved) if reserved else []
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
                    frontier = frontier_from(frozenset())
            if frontier:
                # Same target choice as the built-in sweep: the frontier
                # cell FARTHEST from the goal first, nearest walk distance
                # on ties.
                dist_goal = self._bfs(state, goal, allowed)
                target = min(
                    frontier,
                    key=lambda item: (-dist_goal.get(item[1], 0), item[0]),
                )[1]
                # Gradient descent needs distances measured FROM the
                # target, routed inside the current region.
                dist_target = self._bfs(
                    state, target, region, avoid=reserved | {goal}
                )
                if position not in dist_target:
                    dist_target = self._bfs(state, target, allowed)
                if position not in dist_target:
                    return "stop"
                return self._step_towards(state, dist_target, position, target)

        # 2b. Corridor finale: everything else is swept. Head for the
        #     FARTHEST reserved cell first (the flood-fill 4, the end away
        #     from the goal); the final goal run below then comes back
        #     down: ... 4, 3, 2, 1, goal.
        if reserved:
            unvisited_reserved = [c for c in reserved if c not in visited]
            if unvisited_reserved:
                dist_goal = self._bfs(state, goal, allowed)
                corridor_target = max(
                    unvisited_reserved,
                    key=lambda cell: dist_goal.get(cell, 0),
                )
                dist_corr = self._bfs(
                    state, corridor_target, allowed, avoid={goal}
                )
                if position not in dist_corr:
                    # A pocket reachable only through the goal: fall back
                    # to the open map (crossing the goal mid-sweep is safe
                    # - the run only ends once the corridor is done and
                    # the car is back on the goal).
                    dist_corr = self._bfs(state, corridor_target, allowed)
                if position in dist_corr:
                    return self._step_towards(
                        state, dist_corr, position, corridor_target
                    )

        # 3. Everything reachable is explored: head to the centre goal.
        #    Tie-break matches the built-in final goal run: the smallest
        #    distance wins, then the firmware priority forward > right >
        #    left.
        if position == goal:
            return "stop"
        dist_goal = self._bfs(state, goal, allowed)
        here = dist_goal.get(position, SIZE * SIZE)
        best = None
        for relative in (0, 3, 1, 2):
            d = (state["direction"] + relative) % 4
            if state["walls"][position[1]][position[0]][d]:
                continue
            neighbour = (position[0] + DX[d], position[1] + DY[d])
            value = dist_goal.get(neighbour)
            if value is not None and value < here:
                if best is None or (value, relative) < best[:2]:
                    best = (value, relative, d)
        if best is None:
            return "stop"
        turn = (best[2] - state["direction"]) % 4
        if turn == 0:
            return "forward"
        if turn == 1:
            return "right"
        if turn == 2:
            return "back"
        return "left"
