"""16x16 micromouse maze simulator.

The simulator mirrors the reference firmware's conventions:
N, E, S, W are directions 0..3, and the four centre cells are the goal.

from __future__ import annotations

import importlib.util
import math
import random
import json
import sys
import json
import re
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from tkinter import filedialog, messagebox, simpledialog, ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # pyserial is an optional desktop integration.
    serial = None


SIZE = 16
CELL = 36
PADDING = 22

# Where edited mazes are stored. Wall edits made on the real maze are kept
# in a separate overlay file so "Use real maze" reopens with the user's
# openings intact until they edit them again.
MAZE_SAVE_DIR = Path(__file__).resolve().parent / "saved_mazes"
REAL_MAZE_EDIT_FILE = MAZE_SAVE_DIR / "real_maze_edits.json"
NORTH, EAST, SOUTH, WEST = range(4)
DX = (0, 1, 0, -1)
DY = (-1, 0, 1, 0)
OPPOSITE = (SOUTH, WEST, NORTH, EAST)
GOALS = {(7, 7), (8, 7), (7, 8), (8, 8)}

# Internal adapter-matching keys: the simulation adapters live in folders
# called "ESP32-S3 Micromouse Adapter" / "TQD Micromouse JS2 Adapter".
CAR_CODE_KEYS = {
    "carA": "ESP32-S3 Micromouse",
    "carB": "TQD Micromouse JS2",
}
COLORS = {
    "background": "#0f172a",
    "panel": "#162238",
    "panel_alt": "#1d2d46",
    "canvas": "#0b1324",
    "cell": "#101d33",
    "goal": "#3b3540",
    "wall": "#d9f3ee",
    "flood": "#ffa23e",
    "goal_text": "#ffd166",
    "text": "#e6f4f1",
    "muted": "#9db4c5",
    "accent": "#2dd4bf",
    "accent_dark": "#0f766e",
    "focus": "#5eead4",
    "right_panel": "#cfeeff",
    "right_surface": "#e8f7ff",
    "right_control": "#a9d8f2",
    "right_text": "#0000ff",
    "right_muted": "#0000ff",
    "right_accent": "#0000ff",
}

# Each string describes 16 wall segments. Horizontal strings are boundaries
# between rows 0..15 (top to bottom); vertical strings are boundaries between
# columns 0..15 (left to right), with one string per interior boundary.
# Interior wall edges transcribed from the latest hand-drawn maze reference.
REAL_MAZE_HORIZONTAL = (
    "##.#.###..##....",
    "..#.......#..#..",
    "...#...######.#.",
    ".#.#..#..##.#...",
    ".###.....#.#....",
    "..##......#..#..",
    "...###...#.#.##.",
    "###########.##..",
    ".###.#....##..#.",
    ".#........#.....",
    ".##.#......#...#",
    ".#.#......##.##.",
    ".##.##.####.###.",
    ".#..##..........",
    ".####.#.#######.",
)

REAL_MAZE_VERTICAL = (
    ".######.#.#.##..",
    ".##...#.........",
    "...#.....#.#.#..",
    ".##......###....",
    ".#####...#.#....",
    ".##.###..####...",
    ".#.#.#######.##.",
    "..####.##.###.#.",
    ".##..#######.#..",
    ".#..#.....###.#.",
    "......#......#..",
    ".##.#.#..###.#..",
    ".#.#.#..####.#..",
    ".#####...#.#.#..",
    "##.##.####.#.##.",
)


@dataclass
class Robot:
    x: int = 15
    y: int = 15
    direction: int = WEST


CAR_STARTS = (
    Robot(0, 0, EAST),
    Robot(15, 0, SOUTH),
    Robot(0, 15, NORTH),
    Robot(15, 15, WEST),
)
CAR_COLORS = ("#ff6b6b", "#4dabf7", "#69db7c", "#cc8bff")
CAR_ZONES = (
    (range(0, 8), range(0, 8)),
    (range(8, 16), range(0, 8)),
    (range(0, 8), range(8, 16)),
    (range(8, 16), range(8, 16)),
)
CAR_GOALS = ((7, 7), (8, 7), (7, 8), (8, 8))

# Rendering angle (degrees, 0 = east, y grows downward) per direction.
CAR_ANGLES = (270, 0, 90, 180)

# Cells the car must NOT enter until every other cell of its quadrant is
# swept - the final corridor into its goal. Top cars keep the four cells
# directly ABOVE their goal for the end; bottom cars keep the four cells
# directly BELOW theirs (the red-marked column-8 corridor and its mirrors).
RESERVED_CORRIDOR = {
    0: frozenset({(7, 3), (7, 4), (7, 5), (7, 6)}),
    1: frozenset({(8, 3), (8, 4), (8, 5), (8, 6)}),
    2: frozenset({(7, 9), (7, 10), (7, 11), (7, 12)}),
    3: frozenset({(8, 9), (8, 10), (8, 11), (8, 12)}),
}


class Maze:
    def __init__(self) -> None:
        self.walls: list[list[list[bool]]] = [
            [[False] * 4 for _ in range(SIZE)] for _ in range(SIZE)
        ]
        self.reset_boundaries()

    def reset_boundaries(self) -> None:
        for y in range(SIZE):
            for x in range(SIZE):
                self.walls[y][x] = [False] * 4
                if y == 0:
                    self.walls[y][x][NORTH] = True
                if x == SIZE - 1:
                    self.walls[y][x][EAST] = True
                if y == SIZE - 1:
                    self.walls[y][x][SOUTH] = True
                if x == 0:
                    self.walls[y][x][WEST] = True

    def has_wall(self, x: int, y: int, direction: int) -> bool:
        return self.walls[y][x][direction]

    def set_wall(self, x: int, y: int, direction: int, value: bool) -> None:
        self.walls[y][x][direction] = value
        nx, ny = x + DX[direction], y + DY[direction]
        if 0 <= nx < SIZE and 0 <= ny < SIZE:
            self.walls[ny][nx][OPPOSITE[direction]] = value

    def clear_interior(self) -> None:
        self.reset_boundaries()

    def load_real_maze(self) -> None:
        self.reset_boundaries()
        for boundary_y, row in enumerate(REAL_MAZE_HORIZONTAL, start=1):
            for x, wall in enumerate(row):
                if wall == "#":
                    self.set_wall(x, boundary_y - 1, SOUTH, True)
        for boundary_x, column in enumerate(REAL_MAZE_VERTICAL, start=1):
            for y, wall in enumerate(column):
                if wall == "#":
                    self.set_wall(boundary_x - 1, y, EAST, True)
        self.ensure_quadrant_connectivity()

    def ensure_quadrant_connectivity(self) -> None:
        """Remove interior walls until every quadrant is connected within
        itself: each car must be able to reach every cell of its own
        quadrant without leaving it, so no cell is sealed off behind walls
        that can only be passed from a neighbouring quadrant. Only walls
        BETWEEN two cells of the same quadrant are removed - the quadrant
        boundaries stay exactly as they are."""
        for index in range(4):
            zx, zy = CAR_ZONES[index]
            zone = {(x, y) for x in zx for y in zy}
            while True:
                # Split the zone into components under open internal edges.
                components: list[set[tuple[int, int]]] = []
                left = set(zone)
                while left:
                    seed = next(iter(left))
                    comp = {seed}
                    stack = [seed]
                    while stack:
                        x, y = stack.pop()
                        for d in range(4):
                            if self.walls[y][x][d]:
                                continue
                            n = (x + DX[d], y + DY[d])
                            if n in zone and n not in comp:
                                comp.add(n)
                                stack.append(n)
                    components.append(comp)
                    left -= comp
                if len(components) <= 1:
                    break
                # Open one wall between two touching components and repeat.
                merged = False
                for comp in components:
                    for ax, ay in comp:
                        for d in range(4):
                            n = (ax + DX[d], ay + DY[d])
                            if (
                                n in zone
                                and n not in comp
                                and self.walls[ay][ax][d]
                            ):
                                self.set_wall(ax, ay, d, False)
                                merged = True
                                break
                        if merged:
                            break
                    if merged:
                        break
                if not merged:
                    break

    def randomize(self) -> None:
        self.reset_boundaries()
        # A sparse, connected maze: random internal walls are added only when
        # they do not isolate either adjacent cell from the rest of the grid.
        candidates = [
            (x, y, EAST) for y in range(SIZE) for x in range(SIZE - 1)
        ] + [(x, y, SOUTH) for y in range(SIZE - 1) for x in range(SIZE)]
        random.shuffle(candidates)
        for x, y, direction in candidates:
            if random.random() < 0.22:
                self.set_wall(x, y, direction, True)
        self.ensure_quadrant_connectivity()

    def validate(self) -> list[str]:
        errors: list[str] = []
        for y in range(SIZE):
            for x in range(SIZE):
                for direction in (EAST, SOUTH):
                    nx, ny = x + DX[direction], y + DY[direction]
                    if nx < SIZE and ny < SIZE and self.walls[y][x][direction] != self.walls[ny][nx][OPPOSITE[direction]]:
                        errors.append(f"asymmetric wall at ({x},{y})")
        for x in range(SIZE):
            if not self.walls[0][x][NORTH] or not self.walls[SIZE - 1][x][SOUTH]:
                errors.append("missing north/south boundary")
        for y in range(SIZE):
            if not self.walls[y][0][WEST] or not self.walls[y][SIZE - 1][EAST]:
                errors.append("missing east/west boundary")
        return errors

    def to_dict(self) -> dict:
        return {"size": SIZE, "horizontal": ["".join("#" if self.walls[y][x][SOUTH] else "." for x in range(SIZE)) for y in range(SIZE - 1)],
                "vertical": ["".join("#" if self.walls[y][x][EAST] else "." for y in range(SIZE)) for x in range(SIZE - 1)]}

    def from_dict(self, data: dict) -> None:
        if data.get("size") != SIZE:
            raise ValueError("maze file must describe a 16x16 maze")
        self.reset_boundaries()
        for boundary, row in enumerate(data.get("horizontal", []), 1):
            if len(row) != SIZE:
                raise ValueError("invalid horizontal wall row")
            for x, wall in enumerate(row):
                if wall == "#":
                    self.set_wall(x, boundary - 1, SOUTH, True)
        for boundary, column in enumerate(data.get("vertical", []), 1):
            if len(column) != SIZE:
                raise ValueError("invalid vertical wall column")
            for y, wall in enumerate(column):
                if wall == "#":
                    self.set_wall(boundary - 1, y, EAST, True)

    def analysis(self) -> dict[str, int | bool]:
        reachable = self._reachable((15, 15))
        return {"cells": SIZE * SIZE, "reachable": len(reachable),
                "dead_ends": sum(sum(not self.has_wall(x, y, d) for d in range(4)) == 1
                                  for y in range(SIZE) for x in range(SIZE)),
                "goals_reachable": int(all(goal in reachable for goal in GOALS)),
                "valid": not self.validate()}

    def _reachable(self, start: tuple[int, int]) -> set[tuple[int, int]]:
        seen, queue = {start}, deque([start])
        while queue:
            x, y = queue.popleft()
            for d in range(4):
                nx, ny = x + DX[d], y + DY[d]
                if not self.has_wall(x, y, d) and 0 <= nx < SIZE and 0 <= ny < SIZE and (nx, ny) not in seen:
                    seen.add((nx, ny))
                    queue.append((nx, ny))
        return seen


class Simulator:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("16×16 Micromouse Car Maze Simulator")
        self.root.resizable(True, True)
        self.root.minsize(980, 760)
        self.maze = Maze()
        self.maze.load_real_maze()
        # Which maze is on screen ("real", "generated", "cleared" or a
        # saved file). Wall edits on the real maze are remembered and
        # re-applied every time the real maze is reopened.
        self.maze_origin = "real"
        self.real_maze_edits: dict[str, bool] = {}
        self._load_real_maze_edits()
        self._apply_real_maze_edits()
        self.robot = Robot()
        self.cars: list[Robot] = []
        self.finished: list[int | None] = []
        self.car_known_walls: list[list[list[list[bool]]]] = []
        self.car_visited: list[set[tuple[int, int]]] = []
        self.car_paths: list[list[tuple[int, int]]] = []
        self.car_goal_times: list[int | None] = []
        self.car_exploration_done: list[bool] = []
        self.car_distance: list[int] = []
        self.car_turns: list[int] = []
        # Sensor-beam animation: previous logical pose per car plus the
        # in-flight glide/turn transitions used by the smooth renderer.
        self._last_pose: list[tuple[int, int, int] | None] = [None] * 4
        self._anim: list[dict | None] = [None] * 4
        self.root.after(25, self._render_loop)
        self.race_started_at: float | None = None
        self.race_finished_at: float | None = None
        self.mode = tk.StringVar(value="explore")
        self.speed = tk.IntVar(value=220)
        self.speed_display = tk.StringVar(value="Speed 300  (220 ms)")
        self.turn_delay = tk.BooleanVar(value=True)
        self.realistic_collision = tk.BooleanVar(value=True)
        self.show_trail = tk.BooleanVar(value=True)
        self.show_target_path = tk.BooleanVar(value=True)
        self.sweep_far_first = tk.BooleanVar(value=True)
        self.telemetry = tk.StringVar()
        self.serial_status = tk.StringVar(value="Arduino: unavailable (optional pyserial not installed)" if serial is None else "Arduino: disconnected")
        # Real-car USB link state.
        self.hw_status = tk.StringVar(value="Not paired — scan ports and connect")
        self.hw_battery = tk.StringVar(value="Battery: Not Connected")
        self.hw_info = tk.StringVar(value="")
        self.hw_lines: deque[str] = deque(maxlen=300)
        self.hw_lock = threading.Lock()
        self.hw_serial = None
        self.hw_reader_thread = None
        self.hw_paired = False
        self._hw_battery_next = 0.0
        self.recording = False
        self.recorded_events: list[dict] = []
        self.replay_events: list[dict] = []
        self.replay_index = 0
        self.serial_connection = None
        self.car_type = tk.StringVar(value="builtin")
        # Real car names (from the uploaded code/docs). CAR_CODE_KEYS keeps
        # the internal adapter-matching keys, so uploading still finds the
        # right simulation adapter even though the labels are the real ones.
        self.car_names = {
            slot: tk.StringVar(value=name)
            for slot, name in (
                ("carA", "ESP32-S3 Micromouse"),
                ("carB", "TQD Micromouse JS2"),
                ("own", "My car"),
            )
        }
        self.car_code: dict[str, dict] = {}
        self.car_code_status = {
            slot: tk.StringVar(value="Ready: (Upload Code File)")
            for slot in ("carA", "carB", "own")
        }
        self.controller_cls = None
        self.car_controllers: list = []
        self._autoload_adapters()
        self.flood: dict[tuple[int, int], int] = {}
        self.running = False
        self.moves = 0
        self._build_ui()
        self.reset()

    def _build_ui(self) -> None:
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TFrame", background=COLORS["panel"])
        style.configure("TLabel", background=COLORS["panel"], foreground=COLORS["text"])
        style.configure(
            "TButton", background=COLORS["panel_alt"], foreground=COLORS["text"],
            bordercolor=COLORS["accent_dark"], padding=(8, 5),
        )
        style.configure(
            "Accent.TButton", background=COLORS["accent_dark"],
            foreground="#ffffff", padding=(8, 5),
        )
        style.map(
            "TButton",
            background=[("active", COLORS["accent_dark"]), ("pressed", COLORS["accent"])],
            foreground=[("active", "#ffffff"), ("pressed", "#062c2a")],
        )
        style.map(
            "Accent.TButton",
            background=[("active", COLORS["accent"]), ("pressed", COLORS["focus"])],
            foreground=[("active", "#062c2a"), ("pressed", "#062c2a")],
        )
        style.configure(
            "TRadiobutton", background=COLORS["panel"], foreground=COLORS["text"],
        )
        style.map("TRadiobutton", foreground=[("active", COLORS["focus"])])
        style.configure(
            "TCheckbutton", background=COLORS["panel"], foreground=COLORS["text"],
        )
        style.map("TCheckbutton", foreground=[("active", COLORS["focus"])])
        style.configure(
            "TNotebook", background=COLORS["panel"], borderwidth=0,
        )
        style.configure(
            "TNotebook.Tab", background=COLORS["panel_alt"],
            foreground=COLORS["muted"], padding=(9, 5),
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", COLORS["accent_dark"])],
            foreground=[("selected", "#ffffff")],
        )
        style.configure(
            "TCombobox", fieldbackground=COLORS["panel_alt"],
            background=COLORS["panel_alt"], foreground=COLORS["text"],
        )
        style.configure("Horizontal.TScale", background=COLORS["panel"])
        style.configure("Right.TFrame", background=COLORS["right_panel"])
        style.configure(
            "Right.TLabel", background=COLORS["right_panel"],
            foreground=COLORS["right_text"], font=("Segoe UI", 9, "bold"),
        )
        style.configure(
            "Right.TButton", background=COLORS["right_control"],
            foreground=COLORS["right_text"], bordercolor=COLORS["right_accent"],
            padding=(8, 5), font=("Segoe UI", 9, "bold"),
        )
        style.map(
            "Right.TButton",
            background=[("active", COLORS["right_surface"]), ("pressed", COLORS["right_control"])],
            foreground=[("active", COLORS["right_text"]), ("pressed", COLORS["right_text"])],
        )
        style.configure(
            "Right.TNotebook", background=COLORS["right_panel"], borderwidth=0,
        )
        style.configure(
            "Right.TNotebook.Tab", background=COLORS["right_control"],
            foreground="#000000", padding=(9, 5),
            font=("Segoe UI", 10, "bold"),
        )
        style.map(
            "Right.TNotebook.Tab",
            background=[("selected", COLORS["right_surface"])],
            foreground=[("selected", "#000000")],
        )
        style.configure(
            "Right.TCombobox", fieldbackground=COLORS["right_surface"],
            background=COLORS["right_control"], foreground=COLORS["right_text"],
            font=("Segoe UI", 9, "bold"),
        )
        style.configure(
            "Right.TRadiobutton", background=COLORS["right_panel"],
            foreground="#000000", font=("Segoe UI", 10, "bold"),
        )
        style.configure(
            "Right.TCheckbutton", background=COLORS["right_panel"],
            foreground=COLORS["right_text"], font=("Segoe UI", 9, "bold"),
        )
        style.configure(
            "Right.TSeparator", background=COLORS["right_panel"],
        )
        style.configure("Right.Horizontal.TScale", background=COLORS["right_panel"])
        self.root.configure(background=COLORS["background"])
        container = ttk.Frame(self.root, padding=12)
        container.grid(sticky="nw")
        container.columnconfigure(0, weight=0)
        container.columnconfigure(1, weight=0)
        self.canvas = tk.Canvas(
            container,
            width=PADDING * 2 + SIZE * CELL,
            height=PADDING * 2 + SIZE * CELL,
            background=COLORS["canvas"],
            highlightthickness=0,
        )
        self.canvas.grid(row=0, column=0, rowspan=2, sticky="nw")
        self.canvas.bind("<Button-1>", self._edit_wall)
        self.root.bind("<Key>", self._manual_key)

        panel = tk.Frame(
            container, background=COLORS["right_panel"], padx=14, pady=12,
        )
        panel.grid(row=0, column=1, rowspan=2, sticky="nsw")
        self.panel = panel
        left_panel = tk.Frame(panel, background=COLORS["right_panel"])
        self.left_panel = left_panel
        left_panel.pack(side="left", anchor="nw")
        right_panel = tk.Frame(
            panel, background=COLORS["right_panel"],
            padx=18, pady=12,
        )
        self.right_panel = right_panel
        right_panel.pack(side="left", anchor="nw")
        ttk.Label(
            left_panel, text="Micromouse Car Simulator",
            font=("Segoe UI", 13, "bold"), foreground="#0000ff",
            style="Right.TLabel",
        ).pack(
            anchor="w", pady=(0, 8)
        )
        ttk.Button(
            left_panel, text="Use Real Maze", command=self.load_real_maze,
            style="Right.TButton",
        ).pack(fill="x")
        ttk.Button(
            left_panel, text="Generate Maze", command=self.generate,
            style="Right.TButton",
        ).pack(fill="x", pady=4)
        ttk.Button(
            left_panel, text="Clear Interior Walls", command=self.clear,
            style="Right.TButton",
        ).pack(
            fill="x"
        )
        ttk.Button(
            left_panel, text="Save Maze", command=self.save_maze_to_panel,
            style="Right.TButton",
        ).pack(fill="x", pady=(4, 0))
        ttk.Label(
            left_panel, text="Recent Maze Saves", style="Right.TLabel",
        ).pack(anchor="w", pady=(8, 2))
        self.saved_list = tk.Listbox(
            left_panel,
            height=5,
            width=24,
            activestyle="none",
            background=COLORS["right_panel"],
            foreground=COLORS["right_text"],
            highlightthickness=1,
            borderwidth=0,
            font=("Segoe UI", 9),
            exportselection=False,
        )
        self.saved_list.pack(fill="x")
        self.saved_list.bind("<<ListboxSelect>>", self._load_selected_saved)
        self.saved_list.bind("<Double-Button-1>", self._saved_maze_menu)
        self._refresh_saved_list()
        ttk.Separator(left_panel, style="Right.TSeparator").pack(fill="x", pady=8)
        ttk.Label(
            left_panel, text="Simulation Mode (Four Cars)", style="Right.TLabel",
        ).pack(anchor="w")
        ttk.Radiobutton(
            left_panel,
            text="Full Quadrant, Then Centre",
            variable=self.mode, value="explore",
            command=self.reset,
            style="Right.TRadiobutton",
        ).pack(anchor="w")
        ttk.Radiobutton(
            left_panel,
            text="Shortest Path To Centre",
            variable=self.mode, value="shortest",
            command=self.reset,
            style="Right.TRadiobutton",
        ).pack(anchor="w")
        ttk.Separator(left_panel, style="Right.TSeparator").pack(fill="x", pady=8)
        ttk.Button(
            left_panel, text="Reset Robot", command=self.reset,
            style="Right.TButton",
        ).pack(fill="x")
        self.run_button = ttk.Button(
            left_panel, text="Run", command=self.toggle_run, style="Right.TButton"
        )
        self.run_button.pack(fill="x", pady=4)
        ttk.Button(
            left_panel, text="Step", command=self.step, style="Right.TButton",
        ).pack(fill="x")
        ttk.Button(
            left_panel, text="Recalculate Flood Fill", command=self.recalculate,
            style="Right.TButton",
        ).pack(
            fill="x", pady=4
        )
        notebook = ttk.Notebook(
            right_panel, width=340, height=600, style="Right.TNotebook"
        )
        notebook.pack(anchor="nw")
        self.notebook = notebook

        movement_tab = ttk.Frame(notebook, padding=8, style="Right.TFrame")
        tools_tab = ttk.Frame(notebook, padding=8, style="Right.TFrame")
        status_tab = ttk.Frame(notebook, padding=8, style="Right.TFrame")
        car_tab = ttk.Frame(notebook, padding=8, style="Right.TFrame")
        hardware_tab = ttk.Frame(notebook, padding=8, style="Right.TFrame")
        notebook.add(movement_tab, text="Movement")
        notebook.add(tools_tab, text="Maze Tools")
        notebook.add(status_tab, text="Status")
        notebook.add(car_tab, text="Car Code")
        notebook.add(hardware_tab, text="Hardware")

        ttk.Label(
            movement_tab, text="Run Speed (Increase →)", style="Right.TLabel"
        ).pack(anchor="w")
        speed_row = ttk.Frame(movement_tab, style="Right.TFrame")
        speed_row.pack(fill="x")
        self.speed_slider = tk.IntVar(value=520 - self.speed.get())
        ttk.Scale(
            speed_row, from_=20, to=500, variable=self.speed_slider,
            command=self._update_speed_display,
            orient="horizontal", style="Right.Horizontal.TScale",
        ).pack(side="left", fill="x", expand=True)
        ttk.Label(
            movement_tab, textvariable=self.speed_display,
            style="Right.TLabel",
        ).pack(anchor="e")
        ttk.Checkbutton(
            movement_tab, text="Turn Delay", variable=self.turn_delay,
            style="Right.TCheckbutton",
        ).pack(anchor="w")
        ttk.Checkbutton(
            movement_tab, text="Collision Checking",
            variable=self.realistic_collision,
            style="Right.TCheckbutton",
        ).pack(anchor="w")
        ttk.Checkbutton(
            movement_tab, text="Show Movement Trail",
            variable=self.show_trail,
            command=self.draw,
            style="Right.TCheckbutton",
        ).pack(anchor="w")
        ttk.Checkbutton(
            movement_tab, text="Show Path To Target (Dashed)",
            variable=self.show_target_path,
            command=self.draw,
            style="Right.TCheckbutton",
        ).pack(anchor="w")

        ttk.Label(
            movement_tab, text="Manual Controls: Arrows/WASD\nSpace = Step",
            foreground=COLORS["right_muted"], justify="left", style="Right.TLabel",
        ).pack(anchor="w", pady=(18, 0))

        tools = ttk.Frame(tools_tab, style="Right.TFrame")
        tools.pack(fill="x")
        for column in range(2):
            tools.columnconfigure(column, weight=1)
        ttk.Button(tools, text="Save", command=self.save_maze, style="Right.TButton").grid(
            row=0, column=0, sticky="ew", padx=2, pady=2,
        )
        ttk.Button(tools, text="Load", command=self.load_maze, style="Right.TButton").grid(
            row=0, column=1, sticky="ew", padx=2, pady=2,
        )
        ttk.Button(tools, text="Export", command=self.export_maze, style="Right.TButton").grid(
            row=1, column=0, sticky="ew", padx=2, pady=2,
        )
        ttk.Button(tools, text="Import", command=self.load_maze, style="Right.TButton").grid(
            row=1, column=1, sticky="ew", padx=2, pady=2,
        )
        ttk.Button(
            tools, text="Analyze Maze", command=self.analyze_maze, style="Right.TButton",
        ).grid(row=2, column=0, columnspan=2, sticky="ew", padx=2, pady=2)
        self.record_button = ttk.Button(
            tools, text="Record", command=self.toggle_recording,
            style="Right.TButton",
        )
        self.record_button.grid(
            row=3, column=0, sticky="ew", padx=2, pady=2,
        )
        ttk.Button(tools, text="Replay", command=self.replay, style="Right.TButton").grid(
            row=3, column=1, sticky="ew", padx=2, pady=2,
        )
        ttk.Button(
            tools, text="Save Recording", command=self.save_recording,
            style="Right.TButton",
        ).grid(row=4, column=0, sticky="ew", padx=2, pady=2)
        ttk.Button(
            tools, text="Load Recording", command=self.load_recording,
            style="Right.TButton",
        ).grid(row=4, column=1, sticky="ew", padx=2, pady=2)

        self.status = tk.StringVar()
        self.sensor_text = tk.StringVar()
        self.general_status_widgets: list[tk.Widget] = []
        ttk.Label(
            status_tab, text="Simulation Status", font=("Segoe UI", 10, "bold"),
            style="Right.TLabel",
        ).pack(
            anchor="w"
        )
        ttk.Label(
            status_tab, textvariable=self.status, justify="left",
            wraplength=315, style="Right.TLabel",
        ).pack(
            anchor="w", pady=(6, 12)
        )
        sensor_heading = ttk.Label(
            status_tab, text="Sensors", font=("Segoe UI", 10, "bold"),
            style="Right.TLabel",
        )
        sensor_heading.pack(
            anchor="w"
        )
        sensor_details = ttk.Label(
            status_tab, textvariable=self.sensor_text, justify="left",
            font=("Consolas", 10),
            style="Right.TLabel",
        )
        sensor_details.pack(anchor="w", pady=(4, 8))
        telemetry_label = ttk.Label(
            status_tab, textvariable=self.telemetry, justify="left",
            wraplength=315, foreground=COLORS["right_muted"], style="Right.TLabel",
        )
        telemetry_label.pack(anchor="w")
        self.general_status_widgets = [
            sensor_heading, sensor_details, telemetry_label,
        ]
        self.status_legend = ttk.Label(
            status_tab,
            text=(
                "Click Near A Cell Edge To Toggle A Wall.\n"
                "Orange Color: Flood Value\n"
                "Gold Color: Goal"
            ),
            foreground=COLORS["right_muted"], justify="left",
            wraplength=315, style="Right.TLabel",
        )
        self.status_legend.pack(anchor="w", pady=(18, 0))

        ttk.Label(
            car_tab, text="Car Type (Applies To All 4 Cars)",
            font=("Segoe UI", 10, "bold"), style="Right.TLabel",
        ).pack(anchor="w")
        ttk.Radiobutton(
            car_tab, text="Built-In Scenarios (No Uploaded Code)",
            variable=self.car_type, value="builtin",
            command=self.reset, style="Right.TRadiobutton",
        ).pack(anchor="w", pady=(4, 8))
        for slot in ("carA", "carB", "own"):
            ttk.Radiobutton(
                car_tab, textvariable=self.car_names[slot],
                variable=self.car_type, value=slot,
                command=self.reset, style="Right.TRadiobutton",
            ).pack(anchor="w")
            name_row = ttk.Frame(car_tab, style="Right.TFrame")
            name_row.pack(fill="x", pady=(2, 0))
            ttk.Label(name_row, text="Name:", style="Right.TLabel").pack(side="left")
            ttk.Entry(
                name_row, textvariable=self.car_names[slot], width=18,
            ).pack(side="left", padx=(4, 0))
            ttk.Button(
                name_row, text="Upload Code Folder…",
                command=lambda s=slot: self.upload_car_code(s),
                style="Right.TButton",
            ).pack(side="right")
            ttk.Label(
                car_tab, textvariable=self.car_code_status[slot],
                foreground=COLORS["right_muted"], wraplength=315,
                style="Right.TLabel",
            ).pack(anchor="w", pady=(0, 8))
        ttk.Label(
            car_tab,
            text=(
                "How it works:\n"
                "• Tick/Select the first car model (Built-in Scenarios),\n"
                "  then Run under different conditions — no code upload\n"
                "  needed.\n"
                "• Otherwise, tick/select a car model (ESP32-S3 Micromouse,\n"
                "  TQD Micromouse JS2, or your own car using 'My Car'),\n"
                "  upload the car's corresponding code folder or adapter\n"
                "  file, then Run under different conditions.\n"
                "• You can explore the different panels to see how the\n"
                "  cars and simulations are performing."
            ),
            foreground=COLORS["right_muted"], justify="left",
            wraplength=315, style="Right.TLabel",
        ).pack(anchor="w", pady=(6, 0))

        ttk.Label(
            hardware_tab, text="Real Car Connection",
            font=("Segoe UI", 10, "bold"),
            style="Right.TLabel",
        ).pack(anchor="w")
        ttk.Label(
            hardware_tab, text="Your Software", style="Right.TLabel",
        ).pack(anchor="w", pady=(8, 2))
        self.hw_platform = tk.StringVar(value="Arduino IDE")
        ttk.Combobox(
            hardware_tab, textvariable=self.hw_platform, state="readonly",
            values=(
                "Arduino IDE",
                "Arduino CLI",
                "Arduino Web Editor",
                "PlatformIO (VS Code)",
                "VS Code",
                "Thonny (MicroPython)",
                "Mu Editor (MicroPython)",
                "PyCharm",
                "Keil µVision",
                "STM32CubeIDE",
                "MPLAB X IDE",
                "Tera Term",
                "PuTTY",
            ),
            style="Right.TCombobox",
        ).pack(fill="x")
        ttk.Label(
            hardware_tab, text="USB port", style="Right.TLabel",
        ).pack(anchor="w", pady=(8, 2))
        self.hw_port = tk.StringVar()
        self.hw_port_box = ttk.Combobox(
            hardware_tab, textvariable=self.hw_port, state="readonly",
            values=(), style="Right.TCombobox",
        )
        self.hw_port_box.pack(fill="x")
        hw_buttons = ttk.Frame(hardware_tab, style="Right.TFrame")
        hw_buttons.pack(fill="x", pady=(8, 0))
        ttk.Button(
            hw_buttons, text="Scan Ports", command=self.hw_scan,
            style="Right.TButton",
        ).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        self.hw_connect_button = ttk.Button(
            hw_buttons, text="Connect", command=self.hw_connect,
            style="Right.TButton",
        )
        self.hw_connect_button.grid(row=0, column=1, sticky="ew")
        hw_buttons.columnconfigure(0, weight=1)
        hw_buttons.columnconfigure(1, weight=1)
        ttk.Label(
            hardware_tab, textvariable=self.hw_status,
            wraplength=245, style="Right.TLabel",
        ).pack(anchor="w", pady=(8, 0))
        ttk.Label(
            hardware_tab, textvariable=self.hw_battery,
            wraplength=245, foreground=COLORS["right_muted"],
            style="Right.TLabel",
        ).pack(anchor="w")
        ttk.Label(
            hardware_tab, textvariable=self.hw_info,
            wraplength=245, foreground=COLORS["right_muted"],
            style="Right.TLabel",
        ).pack(anchor="w")
        self.hw_log = tk.Text(
            hardware_tab, height=6, background=COLORS["right_panel"],
            foreground=COLORS["right_text"], borderwidth=0, font=("Segoe UI", 9),
            state="disabled", wrap="word",
        )
        self.hw_log.pack(fill="x")
        ttk.Label(
            hardware_tab,
            text=(
                "The car answers with lines such as {\"name\": \"TQD Micromouse JS2\",\n"
                "\"voltage\": 7.86} (or BAT:7.86V, BAT:87 for percent) — "
                "firmware support required."
            ),
            wraplength=245, foreground=COLORS["right_muted"], justify="left",
            style="Right.TLabel",
        ).pack(anchor="w", pady=(6, 0))

    def _update_speed_display(self, value: str) -> None:
        speed = max(20, int(round(float(value))))
        delay = max(20, 520 - speed)
        self.speed.set(delay)
        self.speed_display.set(f"Speed {speed}  ({delay} ms)")

    def _update_status_panel_size(self) -> None:
        self.notebook.configure(height=600)
        for widget in self.general_status_widgets:
            widget.pack_forget()

    def reset(self) -> None:
        self.running = False
        self.run_button.configure(text="Run")
        self._update_status_panel_size()
        self.robot = Robot()
        self.cars = [Robot(car.x, car.y, car.direction) for car in CAR_STARTS]
        self.finished = [None] * len(self.cars)
        self.car_known_walls = [
            [[[False] * 4 for _ in range(SIZE)] for _ in range(SIZE)]
            for _ in self.cars
        ]
        self.car_visited = [set() for _ in self.cars]
        self.car_paths = [[(car.x, car.y)] for car in self.cars]
        self.car_goal_times = [None] * len(self.cars)
        self.car_exploration_done = [False] * len(self.cars)
        self.car_distance = [0] * len(self.cars)
        self.car_turns = [0] * len(self.cars)
        self.race_started_at = None
        self.race_finished_at = None
        self.moves = 0
        self._setup_controllers()
        self.recalculate()
        self.draw()

    def generate(self) -> None:
        self.running = False
        self.run_button.configure(text="Run")
        self.maze.randomize()
        self.maze_origin = "generated"
        self.reset()

    def clear(self) -> None:
        self.running = False
        self.run_button.configure(text="Run")
        self.maze.clear_interior()
        self.maze_origin = "cleared"
        self.reset()

    def load_real_maze(self) -> None:
        self.running = False
        self.run_button.configure(text="Run")
        self.maze.load_real_maze()
        self.maze_origin = "real"
        # Re-apply the openings/closures the user made earlier: the real
        # maze keeps their edits until they edit it again.
        self._apply_real_maze_edits()
        self.reset()

    # ----- maze persistence -------------------------------------------------

    def _load_real_maze_edits(self) -> None:
        self.real_maze_edits = {}
        try:
            data = json.loads(REAL_MAZE_EDIT_FILE.read_text("utf-8"))
        except (OSError, ValueError):
            return
        if isinstance(data, dict):
            for key, value in data.items():
                self.real_maze_edits[str(key)] = bool(value)

    def _save_real_maze_edits(self) -> None:
        try:
            MAZE_SAVE_DIR.mkdir(parents=True, exist_ok=True)
            REAL_MAZE_EDIT_FILE.write_text(
                json.dumps(self.real_maze_edits), "utf-8"
            )
        except OSError:
            pass

    def _apply_real_maze_edits(self) -> None:
        for key, value in self.real_maze_edits.items():
            try:
                x_text, y_text, d_text = key.split(",")
                x, y, d = int(x_text), int(y_text), int(d_text)
            except ValueError:
                continue
            if 0 <= x < SIZE and 0 <= y < SIZE and 0 <= d < 4:
                self.maze.set_wall(x, y, d, value)

    def _record_wall_edit(self, x: int, y: int, direction: int, value: bool) -> None:
        """Remember an edit made on the real maze so it survives reloads."""
        if self.maze_origin != "real":
            return
        self.real_maze_edits[f"{x},{y},{direction}"] = bool(value)
        self._save_real_maze_edits()

    def save_maze_to_panel(self) -> None:
        """Save the current maze, edits included, into the saves panel."""
        default = time.strftime("maze %H%M")
        name = simpledialog.askstring(
            "Save maze", "Name for this maze:", initialvalue=default,
            parent=self.root,
        )
        if not name:
            return
        name = name.strip() or default
        safe = "".join(
            ch if ch.isalnum() or ch in " -_()" else "_" for ch in name
        )
        path = MAZE_SAVE_DIR / f"{safe}.json"
        payload = {
            "name": name,
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "walls": [
                [
                    [bool(self.maze.walls[y][x][d]) for d in range(4)]
                    for x in range(SIZE)
                ]
                for y in range(SIZE)
            ],
        }
        try:
            MAZE_SAVE_DIR.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload), "utf-8")
        except OSError as error:
            messagebox.showerror("Save maze", f"Could not save: {error}")
            return
        self._refresh_saved_list()

    def _saved_maze_files(self) -> list[Path]:
        try:
            files = [
                path
                for path in MAZE_SAVE_DIR.glob("*.json")
                if path.is_file() and path.name != REAL_MAZE_EDIT_FILE.name
            ]
        except OSError:
            return []
        files.sort(key=lambda path: path.stat().st_mtime, reverse=True)
        return files

    def _refresh_saved_list(self) -> None:
        if not hasattr(self, "saved_list"):
            return
        self.saved_list.delete(0, "end")
        self._saved_files = self._saved_maze_files()
        for path in self._saved_files:
            try:
                stamp = time.strftime(
                    "%d %b %H:%M", time.localtime(path.stat().st_mtime)
                )
            except OSError:
                stamp = ""
            self.saved_list.insert("end", f"{path.stem}   {stamp}")

    def _load_selected_saved(self, _event=None) -> None:
        selection = self.saved_list.curselection()
        if not selection:
            return
        index = selection[0]
        if index >= len(self._saved_files):
            return
        path = self._saved_files[index]
        try:
            payload = json.loads(path.read_text("utf-8"))
            walls = payload["walls"]
        except (OSError, ValueError, KeyError, IndexError):
            messagebox.showerror("Load maze", "That save could not be read.")
            return
        self.running = False
        self.run_button.configure(text="Run")
        for y in range(SIZE):
            for x in range(SIZE):
                for d in range(4):
                    self.maze.walls[y][x][d] = bool(walls[y][x][d])
        self.maze_origin = "saved"
        self.reset()

    def _saved_maze_menu(self, event: tk.Event) -> None:
        """Double-click a saved maze: offer Rename or Delete."""
        index = self.saved_list.nearest(event.y)
        if index < 0 or index >= len(self._saved_files):
            return
        self.saved_list.selection_clear(0, "end")
        self.saved_list.selection_set(index)
        menu = tk.Menu(self.root, tearoff=0)
        menu.add_command(
            label="Rename", command=lambda: self._rename_saved_maze(index)
        )
        menu.add_command(
            label="Delete", command=lambda: self._delete_saved_maze(index)
        )
        try:
            menu.tk_popup(event.x_root, event.y_root)
        finally:
            menu.grab_release()

    def _rename_saved_maze(self, index: int) -> None:
        path = self._saved_files[index]
        new_name = simpledialog.askstring(
            "Rename maze", "New name:", initialvalue=path.stem,
            parent=self.root,
        )
        if not new_name:
            return
        new_name = new_name.strip() or path.stem
        safe = "".join(
            ch if ch.isalnum() or ch in " -_()" else "_" for ch in new_name
        )
        target = MAZE_SAVE_DIR / f"{safe}.json"
        if target.exists() and target != path:
            messagebox.showerror(
                "Rename maze", f"A save named '{safe}' already exists."
            )
            return
        try:
            path.rename(target)
        except OSError as error:
            messagebox.showerror("Rename maze", f"Could not rename: {error}")
            return
        self._refresh_saved_list()
        self.telemetry.set(f"Maze renamed to '{new_name}'")

    def _delete_saved_maze(self, index: int) -> None:
        path = self._saved_files[index]
        if not messagebox.askyesno(
            "Delete maze", f"Delete the saved maze '{path.stem}'?"
        ):
            return
        try:
            path.unlink()
        except OSError as error:
            messagebox.showerror("Delete maze", f"Could not delete: {error}")
            return
        self._refresh_saved_list()
        self.telemetry.set(f"Maze '{path.stem}' deleted")

    def recalculate(self) -> None:
        self.flood = self._calculate_flood(self.car_known_walls[0])
        self.draw()

    def _calculate_flood(
        self, walls: list[list[list[bool]]], targets: set[tuple[int, int]] = GOALS
    ) -> dict[tuple[int, int], int]:
        flood: dict[tuple[int, int], int] = {}
        queue = deque(targets)
        for goal in targets:
            flood[goal] = 0
        while queue:
            x, y = queue.popleft()
            for direction in range(4):
                if walls[y][x][direction]:
                    continue
                nx, ny = x + DX[direction], y + DY[direction]
                if not (0 <= nx < SIZE and 0 <= ny < SIZE):
                    continue
                if (nx, ny) not in flood:
                    flood[(nx, ny)] = flood[(x, y)] + 1
                    queue.append((nx, ny))
        return flood

    def _flood_known_route(
        self,
        walls: list[list[list[bool]]],
        target: tuple[int, int],
        allowed: set[tuple[int, int]],
        visited: set[tuple[int, int]],
    ) -> dict[tuple[int, int], int]:
        """BFS distance to target using only edges the car has sensed open.

        Unknown walls are treated as blocked, and an edge is only trusted
        once at least one of its two cells has been visited (sensed).
        """
        flood: dict[tuple[int, int], int] = {target: 0}
        queue = deque([target])
        while queue:
            x, y = queue.popleft()
            for direction in range(4):
                nx, ny = x + DX[direction], y + DY[direction]
                if (nx, ny) not in allowed or (nx, ny) in flood:
                    continue
                if walls[y][x][direction]:
                    continue
                if (x, y) not in visited and (nx, ny) not in visited:
                    continue
                flood[(nx, ny)] = flood[(x, y)] + 1
                queue.append((nx, ny))
        return flood

    def _autoload_adapters(self) -> None:
        """Pre-wire the ESP32-S3 Micromouse and the TQD Micromouse JS2
        with their adapters so the
        simulation works out of the box. An explicit upload overrides them;
        the "My car" slot always runs only what the user uploads."""
        defaults = dict(CAR_CODE_KEYS)
        for slot, name in defaults.items():
            if slot in self.car_code:
                continue
            adapter = self._find_matching_adapter(name, name)
            if adapter is None:
                continue
            cls, error = self._adapter_import(adapter)
            if cls is None:
                continue
            self.car_code[slot] = {"path": adapter.parent, "cls": cls}
            self.car_code_status[slot].set(
                f"Ready: {adapter.parent.name} "
                "(simulation adapter) — upload to override"
            )

    def upload_car_code(self, slot: str) -> None:
        start_dir = Path(__file__).resolve().parent / "car_codes"
        folder = filedialog.askdirectory(
            title=(
                "Step 1/1: open the car's code folder, then click "
                f"'Select Folder'  —  uploading for {self.car_names[slot].get()}"
            ),
            initialdir=str(start_dir) if start_dir.is_dir() else None,
            mustexist=True,
        )
        if not folder:
            return
        try:
            cls, entry, file_count, note = self._load_controller_code(
                folder,
                hint=CAR_CODE_KEYS.get(slot, self.car_names[slot].get()),
            )
        except Exception as exc:
            messagebox.showerror("Car code load failed", f"{folder}\n\n{exc}")
            return
        self.car_code[slot] = {"path": Path(folder), "cls": cls}
        label = f"Ready: {entry} — Adapter Uploaded"
        if note:
            label = f"{label}\n{note}"
        self.car_code_status[slot].set(label)
        if self.car_type.get() == slot:
            self.reset()
        else:
            self.telemetry.set(
                f"{entry} loaded for {self.car_names[slot].get()}"
            )
        messagebox.showinfo(
            "Car code loaded",
            f"{self.car_names[slot].get()}\n\n"
            f"Folder: {folder}\n"
            f"Entry point: {entry}\n"
            f"Code files found: {file_count}\n"
            + (f"{note}\n" if note else "")
            + "\nNow tick/select its car model below if not already "
            "selected, then click Run.",
        )

    def _adapter_import(self, path: Path):
        """Import a CarController class from a Python adapter file."""
        spec = importlib.util.spec_from_file_location(
            f"user_car_{abs(hash(str(path))) & 0xFFFFFFFF}", path
        )
        if spec is None or spec.loader is None:
            return None, "could not be imported"
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        try:
            spec.loader.exec_module(module)
        except Exception as exc:
            return None, str(exc)
        cls = getattr(module, "CarController", None)
        if cls is None:
            candidates = [
                obj for obj in vars(module).values()
                if isinstance(obj, type)
                and hasattr(obj, "next_move")
                and obj.__module__ == module.__name__
            ]
            if len(candidates) == 1:
                cls = candidates[0]
        if cls is None or not callable(cls) or not hasattr(cls, "next_move"):
            return None, "no CarController class with next_move(state)"
        return cls, ""

    def _find_matching_adapter(
        self, folder_name: str, hint: str = ""
    ) -> Path | None:
        """Match an uploaded code folder to a Python adapter in car_code/.

        Adapter folders are named like "ESP32-S3 Micromouse Adapter"; the
        uploaded folder matches when its normalised name or the selected
        car name contains the adapter's base name. The car's internal key
        is used as an extra hint, so a folder without a telling name
        (e.g. "car2") still matches when "TQD Micromouse JS2" is selected.
        """
        base = Path(__file__).resolve().parent / "car_codes"
        if not base.is_dir():
            return None
        wanted = "".join(
            ch for ch in f"{folder_name}{hint}".lower() if ch.isalnum()
        )
        best = None
        for adapter_dir in base.iterdir():
            if not adapter_dir.is_dir():
                continue
            entry = adapter_dir / "CarController.py"
            if not entry.is_file():
                continue
            core = "".join(
                ch for ch in adapter_dir.name.lower() if ch.isalnum()
            ).replace("adapter", "").replace("codes", "")
            if not core:
                continue
            if core and core in wanted:
                score = len(core)
                if best is None or score > best[0]:
                    best = (score, entry)
        return best[1] if best else None

    def _load_controller_code(self, folder_str: str, hint: str = ""):
        """Load a car's controller from a code folder of any language.

        - Python files defining class CarController run directly.
        - Arduino/C/C++ folders cannot execute inside the Python simulator,
          so the matching Python adapter from car_code/ (or a CarController.py
          placed inside the uploaded folder) is used instead, and this is
          reported back to the UI.
        Returns (cls, entry_name, file_count, note).
        """
        root = Path(folder_str)
        if not root.is_dir():
            raise ValueError("the selected path is not a folder")
        code_suffixes = (".py", ".ino", ".cpp", ".c", ".h", ".hpp", ".mpy")
        code_files = [
            p for p in root.rglob("*")
            if p.is_file() and p.suffix.lower() in code_suffixes
        ]
        if not code_files:
            raise ValueError(
                "no code files found in the selected folder "
                "(.py, .ino, .cpp, .c, .h)"
            )
        py_files = sorted(p for p in code_files if p.suffix.lower() == ".py")
        preferred = ("carcontroller.py", "controller.py", "main.py", "__init__.py")
        top = [p for p in py_files if p.parent == root]
        nested = [p for p in py_files if p.parent != root]
        ordered = (
            [p for p in top if p.name.lower() in preferred]
            + [p for p in nested if p.name.lower() in preferred]
            + [p for p in top if p.name.lower() not in preferred]
            + [p for p in nested if p.name.lower() not in preferred]
        )
        path_added = str(root) not in sys.path
        if path_added:
            sys.path.insert(0, str(root))
        errors = []
        try:
            for path in ordered:
                try:
                    text = path.read_text(encoding="utf-8", errors="ignore")
                except OSError:
                    continue
                if "CarController" not in text and "next_move" not in text:
                    errors.append(f"{path.name}: no CarController class")
                    continue
                cls, error = self._adapter_import(path)
                if cls is not None:
                    return cls, path.name, len(code_files), ""
                errors.append(f"{path.name}: {error}")
        finally:
            if path_added:
                try:
                    sys.path.remove(str(root))
                except ValueError:
                    pass

        # No runnable Python controller: fall back to the car's adapter.
        adapter = self._find_matching_adapter(root.name, hint)
        if adapter is None:
            adapter = next(
                (p for p in root.rglob("CarController.py")), None
            )
        if adapter is not None:
            cls, error = self._adapter_import(adapter)
            if cls is not None:
                has_cc = any(
                    p.suffix.lower() in (".ino", ".cpp", ".c", ".h", ".hpp")
                    for p in code_files
                )
                note = (
                    "C/C++ code detected - running its Python adapter "
                    f"({adapter.parent.name})"
                    if has_cc else
                    f"Using adapter {adapter.parent.name}"
                )
                return cls, adapter.name, len(code_files), note
            errors.append(f"{adapter.name}: {error}")

        detail = "\n".join(errors[:6])
        raise ValueError(
            "no CarController adapter found for this code.\n"
            "Add a CarController.py to the folder, or name the folder so it "
            "matches an adapter in car_codes/."
            + (f"\n\nImport errors:\n{detail}" if detail else "")
        )

    def _setup_controllers(self) -> None:
        info = self.car_code.get(self.car_type.get())
        self.controller_cls = info["cls"] if info else None
        self.car_controllers = []
        for index in range(len(self.cars)):
            controller = None
            if self.controller_cls is not None:
                try:
                    controller = self.controller_cls(index)
                except TypeError:
                    controller = self.controller_cls()
                except Exception as exc:
                    self.telemetry.set(
                        f"Car {index + 1} controller init failed: {exc}"
                    )
            self.car_controllers.append(controller)

    def _car_type_display(self) -> str:
        slot = self.car_type.get()
        if slot == "builtin" or self.controller_cls is None:
            return "built-in logic"
        return self.car_names[slot].get()

    def _controller_move(
        self, index: int, car: Robot, controller: object
    ) -> bool:
        zone_x, zone_y = CAR_ZONES[index]

        def outside(nx: int, ny: int) -> bool:
            """Competition rule: an uploaded car never leaves its quadrant."""
            return nx not in zone_x or ny not in zone_y

        def blocked(direction: int) -> bool:
            nx, ny = car.x + DX[direction], car.y + DY[direction]
            return (
                self.maze.has_wall(car.x, car.y, direction)
                or outside(nx, ny)
            )

        # In the shortest-path scenario the maze is treated as already
        # known, so the controller receives the real wall map; in explore
        # mode it only sees what this car has sensed so far.
        if self.mode.get() == "shortest":
            walls_state = self._quadrant_wall_matrix(index)
        else:
            walls_state = self.car_known_walls[index]

        state = {
            "position": (car.x, car.y),
            "direction": car.direction,
            "sensors": {
                "front": blocked(car.direction),
                "left": blocked((car.direction + 3) % 4),
                "right": blocked((car.direction + 1) % 4),
            },
            "walls": walls_state,
            "visited": self.car_visited[index],
            "goal": CAR_GOALS[index],
            "quadrant": {"x": zone_x, "y": zone_y},
            "scenario": self.mode.get(),
            "sweep_order": "far" if self.sweep_far_first.get() else "near",
            "size": SIZE,
        }
        try:
            action = str(controller.next_move(state)).strip().lower()
        except Exception as exc:
            self.telemetry.set(f"Car {index + 1} code error: {exc}")
            return False
        if action in ("left", "right", "back"):
            turn = {"left": 3, "right": 1, "back": 2}[action]
            car.direction = (car.direction + turn) % 4
            return True
        if action == "stop":
            return False
        if action == "forward":
            nx, ny = car.x + DX[car.direction], car.y + DY[car.direction]
            if outside(nx, ny):
                self.telemetry.set(
                    f"Car {index + 1} held at its quadrant boundary"
                )
                return True
            if self.maze.has_wall(car.x, car.y, car.direction):
                self.telemetry.set(
                    f"Car {index + 1} code tried to drive into a wall"
                )
                return True
            car.x, car.y = nx, ny
            return True
        self.telemetry.set(
            f"Car {index + 1} code returned unknown action {action!r}"
        )
        return False

    def _sweep_complete(self, index: int) -> bool:
        """True when every cell reachable from the car's start has been
        visited, except the centre goal - i.e. the goal run is the only
        remaining part of the explore scenario. Cells behind a pocket that
        is only reachable through the goal count too: the car may cross the
        goal cell while sweeping and only finishes when it returns."""
        zone_x, zone_y = CAR_ZONES[index]
        allowed = set((x, y) for y in zone_y for x in zone_x)
        start = CAR_STARTS[index]
        reachable = self._reachable_exploration_cells(
            (start.x, start.y), allowed, set()
        )
        targets = reachable - {CAR_GOALS[index]}
        return targets.issubset(self.car_visited[index])

    def _reserved(self, index: int) -> frozenset:
        """Cells kept for the final goal approach: the car may not enter
        them until everything else in its quadrant has been swept."""
        return RESERVED_CORRIDOR.get(index, frozenset())

    def _full_wall_matrix(self) -> list:
        """The complete real wall map, in the same format as car_known_walls."""
        return [
            [
                [self.maze.has_wall(x, y, d) for d in range(4)]
                for x in range(SIZE)
            ]
            for y in range(SIZE)
        ]

    def _quadrant_wall_matrix(self, index: int) -> list:
        """Real wall map with the car's quadrant boundary closed, so route
        floods yield the shortest paths that stay inside its own area."""
        zone_x, zone_y = CAR_ZONES[index]
        walls = self._full_wall_matrix()
        for y in range(SIZE):
            for x in range(SIZE):
                inside = x in zone_x and y in zone_y
                for d in range(4):
                    nx, ny = x + DX[d], y + DY[d]
                    if (
                        not inside
                        or nx not in zone_x
                        or ny not in zone_y
                    ):
                        walls[y][x][d] = True
        return walls

    def _planned_route(self, index: int, car: Robot) -> list:
        """The car's current best route from its position to its centre goal.
        In the shortest scenario the real map is known (speed run); in the
        explore scenario the route uses the car's learned walls, falling
        back to an optimistic flood through unknown walls."""
        zone_x, zone_y = CAR_ZONES[index]
        allowed = set((x, y) for y in zone_y for x in zone_x)
        if self.mode.get() == "shortest":
            walls = self._quadrant_wall_matrix(index)
        else:
            walls = self.car_known_walls[index]
        goal = CAR_GOALS[index]
        position = (car.x, car.y)
        if position == goal:
            return []
        known = self._flood_known_route(
            walls, goal, allowed, self.car_visited[index]
        )
        dist = (
            known if position in known
            else self._calculate_flood(walls, {goal})
        )
        route = [position]
        current = position
        for _ in range(SIZE * SIZE):
            if current == goal:
                return route
            here = dist.get(current)
            if here is None:
                break
            best = None
            for direction in range(4):
                if walls[current[1]][current[0]][direction]:
                    continue
                neighbour = (
                    current[0] + DX[direction], current[1] + DY[direction]
                )
                if neighbour not in allowed:
                    continue
                value = dist.get(neighbour)
                if value is not None and value < here:
                    if best is None or value < best[0]:
                        best = (value, neighbour)
            if best is None:
                break
            current = best[1]
            route.append(current)
        return route if route[-1] == goal else []

    def _learn_sensors(
        self, car: Robot, walls: list[list[list[bool]]]
    ) -> None:
        x, y = car.x, car.y
        for direction in range(4):
            wall = self.maze.has_wall(x, y, direction)
            walls[y][x][direction] = wall
            nx, ny = x + DX[direction], y + DY[direction]
            if 0 <= nx < SIZE and 0 <= ny < SIZE:
                walls[ny][nx][OPPOSITE[direction]] = wall

    def _move_car(
        self, index: int, car: Robot, walls: list[list[list[bool]]]
    ) -> bool:
        self._learn_sensors(car, walls)
        visited = self.car_visited[index]
        path = self.car_paths[index]
        position = (car.x, car.y)
        visited.add(position)

        if position in GOALS and self.car_goal_times[index] is None:
            self.car_goal_times[index] = self._elapsed_seconds()

        controller = (
            self.car_controllers[index]
            if index < len(self.car_controllers) else None
        )
        full_exploration = self.mode.get() == "explore"
        # In explore mode crossing the goal cell mid-sweep is allowed (some
        # pockets are only reachable through it); the car only finishes when
        # its quadrant sweep is complete and it is back on the goal.
        exploration_complete = (
            self._sweep_complete(index) if full_exploration else True
        )
        if position == CAR_GOALS[index] and exploration_complete:
            return False
        if self.controller_cls is not None and controller is not None:
            return self._controller_move(index, car, controller)

        zone_x, zone_y = CAR_ZONES[index]
        allowed = set((x, y) for y in zone_y for x in zone_x)
        final_goal = CAR_GOALS[index]
        start = CAR_STARTS[index]

        if exploration_complete and full_exploration:
            # Sweep done: final goal run on the known map. Every step must
            # strictly reduce the distance so the car can never oscillate.
            # Distances are measured with the quadrant boundary closed, so
            # the descent is never lured through another car's quadrant -
            # a shortcut the car is not allowed to follow would leave it
            # with no legal decreasing step and freeze it mid-run.
            closed_walls = [
                [list(self.maze.walls[y][x]) for x in range(SIZE)]
                for y in range(SIZE)
            ]
            for y in range(SIZE):
                for x in range(SIZE):
                    inside = x in zone_x and y in zone_y
                    for d in range(4):
                        nx, ny = x + DX[d], y + DY[d]
                        if (
                            not inside
                            or nx not in zone_x
                            or ny not in zone_y
                        ):
                            closed_walls[y][x][d] = True
            distances = self._calculate_flood(
                closed_walls, {CAR_GOALS[index]}
            )
            here = distances.get(position, SIZE * SIZE)
            candidates = []
            for relative in (0, 3, 1, 2):
                absolute = (car.direction + relative) % 4
                if walls[car.y][car.x][absolute]:
                    continue
                nx, ny = car.x + DX[absolute], car.y + DY[absolute]
                if (nx, ny) in distances and (nx, ny) in allowed:
                    if distances[(nx, ny)] < here:
                        candidates.append(
                            (distances[(nx, ny)], relative, absolute, nx, ny)
                        )
            if not candidates:
                return False
            _, _, absolute, nx, ny = min(candidates)
            if absolute != car.direction and self.turn_delay.get():
                car.direction = absolute
                return True
            car.x, car.y = nx, ny
            car.direction = absolute
            return True

        if full_exploration:
            # Scenario 1: systematic sweep of every reachable quadrant cell.
            # Far cells are cleared first and backtracking always targets the
            # NEAREST cell that still has an unvisited exit, so the car never
            # drifts toward the goal while unexplored cells remain.
            # Learned walls with the quadrant boundary closed. The centre
            # goal cell stays open - a car may cross it while sweeping
            # (some pockets are only reachable through it) and only
            # finishes when its sweep is complete.
            goal_open_map = [
                [list(walls[y][x]) for x in range(SIZE)]
                for y in range(SIZE)
            ]
            for y in range(SIZE):
                for x in range(SIZE):
                    inside = x in zone_x and y in zone_y
                    for d in range(4):
                        nx, ny = x + DX[d], y + DY[d]
                        if (
                            not inside
                            or nx not in zone_x
                            or ny not in zone_y
                        ):
                            goal_open_map[y][x][d] = True
            dist_goal = self._calculate_flood(goal_open_map, {final_goal})

            # Cells held back for the final approach: the car may not step
            # into them, nor route through them, while anything else in the
            # quadrant is still unexplored.
            reserved = self._reserved(index)
            sweep_map = [
                [list(goal_open_map[y][x]) for x in range(SIZE)]
                for y in range(SIZE)
            ]
            blocked = set(reserved)
            if reserved:
                blocked.add(final_goal)
            for rx, ry in blocked:
                for d in range(4):
                    sweep_map[ry][rx][d] = True
                    bx, by = rx + DX[d], ry + DY[d]
                    if 0 <= bx < SIZE and 0 <= by < SIZE:
                        sweep_map[by][bx][OPPOSITE[d]] = True

            # Map for the final corridor run: the corridor itself is open
            # again but the goal stays sealed, so the car descends the
            # corridor and never steps onto the goal early.
            goal_sealed_map = [
                [list(goal_open_map[y][x]) for x in range(SIZE)]
                for y in range(SIZE)
            ]
            if reserved:
                for d in range(4):
                    goal_sealed_map[final_goal[1]][final_goal[0]][d] = True
                    bx = final_goal[0] + DX[d]
                    by = final_goal[1] + DY[d]
                    if 0 <= bx < SIZE and 0 <= by < SIZE:
                        goal_sealed_map[by][bx][OPPOSITE[d]] = True

            # Car 2 clears the top half of its quadrant COMPLETELY before
            # it starts on the bottom half. While a top-half cell is still
            # (optimistically) reachable, the sweep region is restricted to
            # the top half and the walking map seals the boundary so the
            # car cannot drift down. The phase ends as soon as no top-half
            # cell is reachable any more (walls learned / only through the
            # corridor), and the normal whole-quadrant sweep resumes.
            top_mid = (min(zone_y) + max(zone_y) + 1) // 2
            top_allowed = {c for c in allowed if c[1] < top_mid}
            top_unvisited = [
                c for c in top_allowed
                if c not in visited
                and c != final_goal
                and c not in reserved
            ]
            top_phase = False
            phase_map = None
            # A borrowed cell: the car stepped one cell over the boundary
            # to finish a corridor and now walks straight back up.
            borrowed = bool(getattr(car, "phase_borrowed", False))
            if index == 1 and top_unvisited:
                phase_map = [
                    [list(sweep_map[y][x]) for x in range(SIZE)]
                    for y in range(SIZE)
                ]
                for x in range(SIZE):
                    phase_map[top_mid][x][NORTH] = True
                    phase_map[top_mid - 1][x][SOUTH] = True
                reach = self._calculate_flood(phase_map, {position})
                if any(c in reach for c in top_unvisited):
                    top_phase = True
            if borrowed and index == 1 and top_unvisited:
                top_phase = True
            if top_phase:
                region = top_allowed
            else:
                region = allowed

            # 1. Take a direction that leads somewhere new. Order: by
            #    default the FARTHEST cells from the goal are cleared
            #    first, so the goal area (and the final goal run) comes
            #    last. The reserved corridor is never entered while
            #    anything else is unexplored.
            candidates = []
            for relative in (0, 3, 1, 2):
                absolute = (car.direction + relative) % 4
                if walls[car.y][car.x][absolute]:
                    continue
                nx, ny = car.x + DX[absolute], car.y + DY[absolute]
                if (nx, ny) not in region:
                    continue
                if (nx, ny) == final_goal:
                    continue
                if (nx, ny) in reserved:
                    continue
                if (nx, ny) not in visited:
                    candidates.append((relative, absolute, nx, ny))
            crossing = False
            if not candidates and top_phase and not borrowed:
                # Straight continuation over the top/bottom boundary of
                # the phase: when the corridor the car is already driving
                # in carries on into the other half, take that cell now
                # instead of driving all the way back for it later. It is
                # a single borrowed cell - the car then climbs straight
                # back into the top half to finish the phase.
                absolute = car.direction
                if not walls[car.y][car.x][absolute]:
                    nx, ny = car.x + DX[absolute], car.y + DY[absolute]
                    if (
                        (nx, ny) in allowed
                        and (nx, ny) not in visited
                        and (nx, ny) != final_goal
                        and (nx, ny) not in reserved
                    ):
                        candidates.append((0, absolute, nx, ny))
                        crossing = True
            if candidates:
                if self.sweep_far_first.get():
                    # Straight ahead first: while the car is driving into
                    # unexplored ground it finishes the cell right in front
                    # of it instead of turning away and having to come back
                    # for it later. Farther-from-goal cells win otherwise.
                    candidates.sort(
                        key=lambda item: (
                            item[0] != 0,
                            -dist_goal.get(
                                (item[2], item[3]), SIZE * SIZE
                            ),
                            item[0],
                        )
                    )
                _, absolute, nx, ny = candidates[0]
                if absolute != car.direction:
                    car.direction = absolute
                    return True
                if crossing:
                    car.phase_borrowed = True
                elif ny < top_mid:
                    car.phase_borrowed = False
                car.x, car.y = nx, ny
                car.direction = absolute
                return True

            # 2. No unexplored neighbour: walk to the farthest remaining
            #    frontier, never routing through the reserved corridor.
            def frontier_cells(walk_map, region) -> list:
                from_here = self._calculate_flood(walk_map, {position})
                found = []
                for (cx, cy), value in from_here.items():
                    for d in range(4):
                        if walls[cy][cx][d]:
                            continue
                        neighbour = (cx + DX[d], cy + DY[d])
                        if neighbour not in region:
                            continue
                        if neighbour == final_goal or neighbour in visited:
                            continue
                        if neighbour in reserved:
                            continue
                        found.append(
                            (value, -dist_goal.get((cx, cy), 0), (cx, cy))
                        )
                        break
                return found

            if top_phase:
                # A borrowed car sits below the sealed boundary: walk it
                # back up on the open map, still aiming inside the top
                # half so the phase can be finished.
                walk_map = sweep_map if borrowed else phase_map
                frontier = frontier_cells(walk_map, region)
            else:
                walk_map = sweep_map
                frontier = frontier_cells(sweep_map, allowed)
            if not frontier:
                # Nothing left outside the reserved corridor. If some cells
                # can only be reached through it, sweep them first; other-
                # wise the final goal run below drives through it.
                leftover = allowed - visited - reserved - {final_goal}
                if leftover:
                    frontier = frontier_cells(goal_open_map, allowed)
                    walk_map = goal_open_map
            if not frontier and reserved:
                # Everything else is swept: now drive INTO the held-back
                # corridor. Head for the FARTHEST cell of the corridor
                # first (the flood-fill 4, the end away from the goal) and
                # only then come back down: ... 4, 3, 2, 1, goal.
                corridor_target = None
                for cell in reserved:
                    if cell in visited:
                        continue
                    if corridor_target is None or dist_goal.get(
                        cell, 0
                    ) > dist_goal.get(corridor_target, 0):
                        corridor_target = cell
                if corridor_target is not None:
                    dist_corr = self._calculate_flood(
                        goal_sealed_map, {corridor_target}
                    )
                    here_value = dist_corr.get(position)
                    if here_value is None:
                        # A pocket reachable only through the goal can leave
                        # the car on the far side of it: fall back to the
                        # open map. Crossing the goal mid-sweep is safe -
                        # retirement stays blocked until the corridor is
                        # done and the car has returned to the goal.
                        dist_corr = self._calculate_flood(
                            goal_open_map, {corridor_target}
                        )
                        here_value = dist_corr.get(position)
                    if here_value is not None:
                        best = None
                        for relative in (0, 3, 1, 2):
                            absolute = (car.direction + relative) % 4
                            if walls[car.y][car.x][absolute]:
                                continue
                            neighbour = (
                                car.x + DX[absolute],
                                car.y + DY[absolute],
                            )
                            if neighbour not in allowed:
                                continue
                            value = dist_corr.get(neighbour)
                            if value is not None and value < here_value:
                                if best is None or value < best[0]:
                                    best = (value, absolute)
                        if best is not None:
                            _, absolute = best
                            if absolute != car.direction:
                                car.direction = absolute
                                return True
                            car.x = car.x + DX[absolute]
                            car.y = car.y + DY[absolute]
                            car.direction = absolute
                            return True
            if frontier:
                # Farthest remaining frontier first: the deepest cells are
                # always finished before the car ever comes near the goal.
                target = min(
                    frontier, key=lambda item: (item[1], item[0])
                )[2]
                dist_target = self._calculate_flood(walk_map, {target})
                here_value = dist_target.get(position)
                if here_value is None and top_phase:
                    # Route on the open quadrant map rather than ending the
                    # run - the target may sit behind not-yet-learned walls.
                    dist_target = self._calculate_flood(
                        goal_open_map, {target}
                    )
                    here_value = dist_target.get(position)
                if here_value is None:
                    return False
                best = None
                for relative in (0, 3, 1, 2):
                    absolute = (car.direction + relative) % 4
                    if walls[car.y][car.x][absolute]:
                        continue
                    neighbour = (
                        car.x + DX[absolute], car.y + DY[absolute]
                    )
                    if neighbour not in allowed:
                        continue
                    value = dist_target.get(neighbour)
                    if value is not None and value < here_value:
                        if best is None or value < best[0]:
                            best = (value, absolute)
                if best is None:
                    return False
                _, absolute = best
                if absolute != car.direction:
                    car.direction = absolute
                    return True
                car.x, car.y = car.x + DX[absolute], car.y + DY[absolute]
                car.direction = absolute
                return True

            # Frontier empty: sweep is done, drive the final goal run.
            dist_final = self._calculate_flood(goal_open_map, {final_goal})
            here_value = dist_final.get(position)
            if here_value is None:
                return False
            best = None
            for relative in (0, 3, 1, 2):
                absolute = (car.direction + relative) % 4
                if walls[car.y][car.x][absolute]:
                    continue
                neighbour = (car.x + DX[absolute], car.y + DY[absolute])
                if neighbour not in allowed:
                    continue
                value = dist_final.get(neighbour)
                if value is not None and value < here_value:
                    if best is None or value < best[0]:
                        best = (value, absolute)
            if best is None:
                return False
            _, absolute = best
            if absolute != car.direction:
                car.direction = absolute
                return True
            car.x, car.y = car.x + DX[absolute], car.y + DY[absolute]
            car.direction = absolute
            return True

        # Scenario 2: speed run on the known map. The maze is treated as
        # already known (like the racing run after exploration), so the car
        # drives the TRUE shortest route to its centre goal and visits only
        # the cells on that route - no sweeping, no detours.
        real_walls = self._quadrant_wall_matrix(index)
        flood = self._calculate_flood(real_walls, {final_goal})
        current = flood.get(position, SIZE * SIZE)
        best = None
        for relative in (0, 3, 1, 2):
            absolute = (car.direction + relative) % 4
            if self.maze.has_wall(car.x, car.y, absolute):
                continue
            nx, ny = car.x + DX[absolute], car.y + DY[absolute]
            if (nx, ny) not in allowed:
                continue
            value = flood.get((nx, ny), SIZE * SIZE)
            if value < current and (best is None or value < best[0]):
                best = (value, absolute, nx, ny)
        if best is None:
            return False
        _, absolute, nx, ny = best
        if absolute != car.direction and self.turn_delay.get():
            car.direction = absolute
            return True
        path.append((nx, ny))
        car.x, car.y = nx, ny
        car.direction = absolute
        return True

    def _reachable_exploration_cells(
        self,
        start: tuple[int, int],
        allowed: set[tuple[int, int]],
        excluded: set[tuple[int, int] | None],
    ) -> set[tuple[int, int]]:
        reachable = {start}
        queue = deque([start])
        while queue:
            x, y = queue.popleft()
            for direction in range(4):
                nx, ny = x + DX[direction], y + DY[direction]
                if (
                    (nx, ny) in allowed
                    and (nx, ny) not in excluded
                    and (nx, ny) not in reachable
                    and not self.maze.has_wall(x, y, direction)
                ):
                    reachable.add((nx, ny))
                    queue.append((nx, ny))
        return reachable

    def _manual_key(self, event: tk.Event) -> None:
        # Typing in a text field (e.g. renaming a car) must never move cars.
        focus = self.root.focus_get()
        if isinstance(
            focus,
            (tk.Entry, ttk.Entry, ttk.Spinbox, ttk.Combobox, tk.Text),
        ):
            return
        if self.cars:
            self.robot = self.cars[0]
        key = event.keysym.lower()
        if key == "space":
            self.step()
            return
        direction = {"up": NORTH, "w": NORTH, "right": EAST, "d": EAST,
                     "down": SOUTH, "s": SOUTH, "left": WEST, "a": WEST}.get(key)
        if direction is None:
            return
        self.robot.direction = direction
        if self.realistic_collision.get() and self.maze.has_wall(self.robot.x, self.robot.y, direction):
            self.telemetry.set("Manual move blocked by wall")
        else:
            nx, ny = self.robot.x + DX[direction], self.robot.y + DY[direction]
            if 0 <= nx < SIZE and 0 <= ny < SIZE:
                self.robot.x, self.robot.y = nx, ny
                self.moves += 1
                self._record_frame()
        self.draw()

    def _record_frame(self) -> None:
        if self.recording:
            self.recorded_events.append({
                "time": time.time(),
                "cars": [
                    {"x": car.x, "y": car.y, "direction": car.direction}
                    for car in self.cars
                ],
                "finished": list(self.finished),
            })

    def toggle_recording(self) -> None:
        if not self.recording:
            self.recording = True
            self.recorded_events = []
            self._record_frame()
            self.record_button.configure(text="Stop Recording")
            self.telemetry.set("Simulation recording started")
            return

        self.recording = False
        self.record_button.configure(text="Record")
        frame_count = len(self.recorded_events)
        self.telemetry.set(f"Recording stopped ({frame_count} frames)")
        if not frame_count:
            return
        save = messagebox.askyesno(
            "Recording complete",
            f"Recording captured {frame_count} frames.\n\nSave it now?",
        )
        if save:
            self.save_recording()
        replay_now = messagebox.askyesno(
            "Replay recording",
            "Do you want to replay this recording now?\n\n"
            "Choose No to replay it later from Maze tools.",
        )
        if replay_now:
            self.replay()

    def replay(self) -> None:
        if not self.recorded_events:
            self.telemetry.set("No recording available")
            return
        self.replay_events = list(self.recorded_events)
        self.replay_index = 0
        self.reset()
        self._replay_tick()

    def _replay_tick(self) -> None:
        if self.replay_index >= len(self.replay_events):
            self.telemetry.set("Replay complete")
            return
        event = self.replay_events[self.replay_index]
        cars = event.get("cars", [])
        if len(cars) != len(self.cars):
            self.telemetry.set("Replay does not match the current simulation mode")
            return
        for car, saved in zip(self.cars, cars):
            car.x = saved["x"]
            car.y = saved["y"]
            car.direction = saved["direction"]
        self.finished = list(event.get("finished", self.finished))
        self.robot = self.cars[0]
        self.replay_index += 1
        self.draw()
        self.root.after(max(20, int(self.speed.get())), self._replay_tick)

    def save_recording(self) -> bool:
        if not self.recorded_events:
            self.telemetry.set("No recording available")
            return False
        name = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=(("Simulation recording", "*.json"),),
        )
        if name:
            data = {
                "mode": self.mode.get(),
                "maze": self.maze.to_dict(),
                "frames": self.recorded_events,
            }
            Path(name).write_text(json.dumps(data, indent=2), encoding="utf-8")
            self.telemetry.set(f"Saved recording {Path(name).name}")
            return True
        return False

    def load_recording(self) -> None:
        name = filedialog.askopenfilename(
            filetypes=(("Simulation recording", "*.json"),),
        )
        if not name:
            return
        try:
            data = json.loads(Path(name).read_text(encoding="utf-8"))
            frames = data.get("frames")
            if not isinstance(frames, list) or not frames:
                raise ValueError("recording contains no simulation frames")
            self.maze.from_dict(data["maze"])
            saved_mode = data.get("mode", "explore")
            self.mode.set(
                saved_mode if saved_mode in ("explore", "shortest") else "explore"
            )
            self.recorded_events = frames
            self.reset()
            self.telemetry.set(f"Loaded recording {Path(name).name}")
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            messagebox.showerror("Recording load failed", str(exc))

    def save_maze(self) -> None:
        name = filedialog.asksaveasfilename(defaultextension=".json",
                                            filetypes=(("Maze JSON", "*.json"),))
        if name:
            Path(name).write_text(json.dumps(self.maze.to_dict(), indent=2), encoding="utf-8")
            self.telemetry.set(f"Saved {Path(name).name}")

    def load_maze(self) -> None:
        name = filedialog.askopenfilename(filetypes=(("Maze JSON", "*.json"),))
        if name:
            try:
                self.maze.from_dict(json.loads(Path(name).read_text(encoding="utf-8")))
                self.reset()
                self.telemetry.set(f"Loaded {Path(name).name}")
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                messagebox.showerror("Maze load failed", str(exc))

    export_maze = save_maze

    def analyze_maze(self) -> None:
        info = self.maze.analysis()
        problems = self.maze.validate()
        messagebox.showinfo("Maze analysis", "\n".join(
            f"{key.replace('_', ' ').title()}: {value}" for key, value in info.items()
        ) + ("\n\nValidation:\n" + "\n".join(problems[:8]) if problems else "\n\nValidation: OK"))

    # ----- real car USB link -------------------------------------------------

    def hw_scan(self) -> None:
        """List the USB serial ports the car could be attached to."""
        if serial is None:
            self.hw_status.set("pyserial is not installed — cannot scan ports")
            return
        ports = serial.tools.list_ports.comports()
        if not ports:
            self.hw_port_box.configure(values=())
            self.hw_status.set("No USB serial ports found — plug the car in")
            return
        entries = [
            f"{port.device} — {port.description}" for port in ports
        ]
        self.hw_port_box.configure(values=entries)
        self.hw_port.set(entries[0])
        self.hw_status.set(f"{len(entries)} port(s) found — select and connect")

    def hw_connect(self) -> None:
        if serial is None:
            self.hw_status.set("pyserial is not installed — cannot connect")
            return
        selection = self.hw_port.get()
        if not selection:
            self.hw_status.set("Scan ports first, then select the car's port")
            return
        device = selection.split(" — ")[0].strip()
        try:
            self.hw_serial = serial.Serial(device, 115200, timeout=0.2)
        except Exception as exc:
            self.hw_status.set(f"Connection failed: {exc}")
            return
        self.hw_paired = False
        self.hw_lines.clear()
        self._hw_log_line(f"--- connected to {device} ---")
        # Handshake: ask the car to introduce itself. Firmware that knows
        # the protocol replies with a JSON line (name/battery/version).
        if "MicroPython" in self.hw_platform.get():
            self.hw_serial.write(b"\r\x03\r\x03")
            time.sleep(0.1)
            self.hw_serial.write(
                b'try:\n import hw_info\n print("@@INFO@" + json.dumps(hw_info.info()))\nexcept:\n print("@@INFO@" + json.dumps({"name": "MicroPython car"}))\r\n'
            )
        else:
            self.hw_serial.write(b"INFO\n")
        self.hw_serial.flush()
        self.hw_reader_thread = threading.Thread(
            target=self._hw_reader_loop, daemon=True
        )
        self.hw_reader_thread.start()
        self.hw_poll_after_id = self.root.after(150, self._hw_poll)
        self.root.after(2500, self._hw_pairing_check)
        self.hw_connect_button.configure(text="Disconnect", command=self.hw_disconnect)

    def _hw_pairing_check(self) -> None:
        if self.hw_serial is None:
            return
        if self.hw_paired:
            self.serial_status.set("Arduino: paired")
        else:
            self.hw_status.set(
                f"Connected to {self.hw_serial.port} — no car data received "
                "(firmware must print its info/battery lines)"
            )
            self.serial_status.set(f"Arduino: connected to {self.hw_serial.port} (not paired)")

    def _hw_reader_loop(self) -> None:
        ser = self.hw_serial
        while ser is not None and ser.is_open:
            try:
                raw = ser.readline()
            except Exception:
                break
            if raw:
                line = raw.decode("utf-8", "replace").strip()
                if line:
                    with self.hw_lock:
                        self.hw_lines.append(line)

    def _hw_poll(self) -> None:
        if self.hw_serial is None:
            return
        fresh = []
        with self.hw_lock:
            while self.hw_lines:
                fresh.append(self.hw_lines.popleft())
        for line in fresh:
            self._hw_log_line(line)
            self._hw_parse_line(line)
        if time.monotonic() >= self._hw_battery_next:
            self._hw_battery_next = time.monotonic() + 2.0
            self._hw_request_battery()
        self.hw_poll_after_id = self.root.after(150, self._hw_poll)

    def _hw_parse_line(self, line: str) -> None:
        payload = None
        marker = "@@INFO@"
        if marker in line:
            try:
                payload = json.loads(line.split(marker, 1)[1].strip())
            except ValueError:
                payload = None
        elif line.lstrip().startswith("{"):
            try:
                payload = json.loads(line)
            except ValueError:
                payload = None
        if isinstance(payload, dict):
            self.hw_paired = True
            name = payload.get("name", "car")
            version = payload.get("version", "")
            self.hw_info.set(
                f"Paired with: {name}" + (f" (firmware {version})" if version else "")
            )
            volts = payload.get("voltage")
            if volts is not None:
                try:
                    volts = float(volts)
                    if volts > 50:  # reported in millivolts
                        volts /= 1000.0
                except (TypeError, ValueError):
                    volts = None
            percent = payload.get("battery")
            if percent is not None:
                try:
                    percent = float(percent)
                except (TypeError, ValueError):
                    percent = None
            self._hw_set_battery(volts, percent)
        else:
            match = re.search(
                r"BAT(?:TERY)?\s*[:=]\s*([\d.]+)\s*(V|%)?", line, re.I
            )
            if match:
                self.hw_paired = True
                value = float(match.group(1))
                unit = (match.group(2) or "").lower()
                if unit == "v" or value <= 12.6:
                    self._hw_set_battery(volts=value)
                else:
                    self._hw_set_battery(percent=value)

    def _hw_set_battery(self, volts=None, percent=None) -> None:
        """Show the REAL battery reading from the connected car: volts when
        the firmware reports them, otherwise the percentage plus the
        estimated pack voltage - always with a charge-state verdict."""
        if volts is not None:
            shown = f"{volts:.2f} V"
        elif percent is not None:
            shown = f"{percent:.0f}%  (~{6.0 + 0.024 * percent:.2f} V)"
        else:
            return
        level = volts if volts is not None else 6.0 + 0.024 * (percent or 0)
        if level >= 8.2:
            state = "Fully Charged"
        elif level < 6.8:
            state = "Needs Charging"
        else:
            state = "Partly Charged"
        self.hw_battery.set(f"Battery: {shown} — {state}")

    def _hw_log_line(self, line: str) -> None:
        self.hw_log.configure(state="normal")
        self.hw_log.insert("end", line + "\n")
        self.hw_log.delete("1.0", "2.0") if int(
            self.hw_log.index("end-1c").split(".")[0]
        ) > 40 else None
        self.hw_log.configure(state="disabled")

    def _hw_request_battery(self) -> None:
        if self.hw_serial is None or not self.hw_serial.is_open:
            return
        try:
            if "MicroPython" in self.hw_platform.get():
                self.hw_serial.write(
                    b'try:\n import hw_info\n print("@@INFO@" + json.dumps({"battery": hw_info.battery()}))\nexcept:\n pass\r\n'
                )
            else:
                self.hw_serial.write(b"BATTERY\n")
            self.hw_serial.flush()
        except Exception:
            pass

    def hw_disconnect(self) -> None:
        ser, self.hw_serial = self.hw_serial, None
        if getattr(self, "hw_poll_after_id", None):
            try:
                self.root.after_cancel(self.hw_poll_after_id)
            except Exception:
                pass
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        self.hw_paired = False
        self.hw_status.set("Disconnected")
        self.hw_battery.set("Battery: Not Connected")
        self.hw_info.set("")
        self.serial_status.set("Arduino: disconnected")
        self.hw_connect_button.configure(text="Connect", command=self.hw_connect)

    def _edit_wall(self, event: tk.Event) -> None:
        x = int((event.x - PADDING) // CELL)
        y = int((event.y - PADDING) // CELL)
        if not (0 <= x < SIZE and 0 <= y < SIZE):
            return
        local_x = (event.x - PADDING) % CELL
        local_y = (event.y - PADDING) % CELL
        distances = {
            WEST: local_x,
            EAST: CELL - local_x,
            NORTH: local_y,
            SOUTH: CELL - local_y,
        }
        direction = min(distances, key=distances.get)
        if distances[direction] > 8:
            return
        new_value = not self.maze.has_wall(x, y, direction)
        self.maze.set_wall(x, y, direction, new_value)
        self._record_wall_edit(x, y, direction, new_value)
        self.recalculate()

    def _sensor(self, relative_direction: int) -> bool:
        absolute = (self.robot.direction + relative_direction) % 4
        return self.maze.has_wall(self.robot.x, self.robot.y, absolute)

    def _car_sensor(self, car: Robot, relative_direction: int) -> bool:
        absolute = (car.direction + relative_direction) % 4
        return self.maze.has_wall(car.x, car.y, absolute)

    def step(self) -> None:
        active = False
        for index, car in enumerate(self.cars):
            if self.finished[index] is not None:
                continue
            active = True
            walls = self.car_known_walls[index]
            previous_position = (car.x, car.y)
            previous_direction = car.direction
            if not self._move_car(index, car, walls):
                self.finished[index] = self.moves + 1
            if (car.x, car.y) != previous_position:
                self.car_distance[index] += 1
            if car.direction != previous_direction:
                self.car_turns[index] += 1
        self.moves += 1
        self.robot = self.cars[0]
        self._record_frame()
        self.recalculate()
        if not active or all(value is not None for value in self.finished):
            self.race_finished_at = time.monotonic()
            self.running = False
            self.run_button.configure(text="Run")
            results = ", ".join(
                f"Car {i + 1}: {self._competition_result(i)}"
                for i in range(len(self.cars))
            )
            messagebox.showinfo("Competition finished", results)

    def toggle_run(self) -> None:
        self.running = not self.running
        if self.running and self.race_started_at is None:
            self.race_started_at = time.monotonic()
        self.run_button.configure(text="Pause" if self.running else "Run")
        if self.running:
            self._run_tick()

    def _run_tick(self) -> None:
        if not self.running:
            return
        self.step()
        if self.running:
            self.root.after(max(20, int(self.speed.get())), self._run_tick)

    def _render_loop(self) -> None:
        """Continuous redraw so the glide/turn animations stay smooth."""
        try:
            self.draw()
        except tk.TclError:
            return  # window is closing
        self.root.after(25, self._render_loop)

    def draw(self) -> None:
        self.canvas.delete("all")
        for y in range(SIZE):
            for x in range(SIZE):
                left = PADDING + x * CELL
                top = PADDING + y * CELL
                right, bottom = left + CELL, top + CELL
                self.canvas.create_rectangle(
                    left + 1, top + 1, right - 1, bottom - 1,
                    fill=COLORS["goal"] if (x, y) in GOALS else COLORS["cell"],
                    outline="",
                )
                if (x, y) in self.flood:
                    self.canvas.create_text(
                        (left + right) / 2, (top + bottom) / 2,
                        text=str(self.flood[(x, y)]),
                        fill=COLORS["flood"] if (x, y) not in GOALS else COLORS["goal_text"],
                        font=("Segoe UI", 9),
                    )
                for direction in (NORTH, EAST, SOUTH, WEST):
                    if self.maze.has_wall(x, y, direction):
                        x1, y1, x2, y2 = left, top, right, bottom
                        if direction == NORTH:
                            x2, y2 = right, top
                        elif direction == EAST:
                            x1, y1, x2, y2 = right, top, right, bottom
                        elif direction == SOUTH:
                            x1, y1, x2, y2 = left, bottom, right, bottom
                        else:
                            x1, y1, x2, y2 = left, top, left, bottom
                        self.canvas.create_line(
                            x1, y1, x2, y2, fill=COLORS["wall"], width=4
                        )
        # "GOAL" across the centre block - shown until the first car gets
        # there, so the cars themselves stay readable afterwards.
        if all(t is None for t in self.car_goal_times):
            centre = PADDING + 8 * CELL
            self.canvas.create_text(
                centre + 1, centre + 1, text="GOAL",
                fill="#7c5c12", font=("Segoe UI", 13, "bold"),
            )
            self.canvas.create_text(
                centre, centre, text="GOAL",
                fill=COLORS["goal_text"], font=("Segoe UI", 13, "bold"),
            )
        if self.show_trail.get():
            for index in range(len(self.cars)):
                for x, y in self.car_visited[index]:
                    left = PADDING + x * CELL
                    top = PADDING + y * CELL
                    self.canvas.create_rectangle(
                        left + 3, top + 3, left + CELL - 3, top + CELL - 3,
                        fill=CAR_COLORS[index], outline="", stipple="gray50",
                    )
        if self.show_target_path.get():
            for index, car in enumerate(self.cars):
                # Only visible once that car actually starts to move.
                if len(self.car_visited[index]) <= 1:
                    continue
                route = self._planned_route(index, car)
                if len(route) < 2:
                    continue
                points = []
                for x, y in route:
                    points += [
                        PADDING + (x + 0.5) * CELL,
                        PADDING + (y + 0.5) * CELL,
                    ]
                self.canvas.create_line(
                    *points, fill=CAR_COLORS[index], width=2, dash=(5, 4)
                )
        # Cars are rendered at interpolated poses so they glide cell to
        # cell and rotate smoothly, like real cars, instead of jumping.
        now = time.monotonic()
        anim_ms = max(50.0, float(self.speed.get())) / 1000.0
        for index, car in enumerate(self.cars):
            pose = (car.x, car.y, car.direction)
            previous = self._last_pose[index] if index < len(self._last_pose) else None
            if previous is not None and pose != previous:
                kind = (
                    "turn" if (pose[0], pose[1]) == (previous[0], previous[1])
                    else "move"
                )
                self._anim[index] = {
                    "kind": kind,
                    "from": previous,
                    "t0": now,
                }
            self._last_pose[index] = pose
            motion, phase = "idle", 0.0
            fx, fy = float(car.x), float(car.y)
            angle = CAR_ANGLES[car.direction]
            anim = self._anim[index] if index < len(self._anim) else None
            if anim is not None:
                progress = (now - anim["t0"]) / anim_ms
                if progress >= 1.0:
                    self._anim[index] = None
                else:
                    motion = anim["kind"]
                    eased = progress * progress * (3 - 2 * progress)
                    phase = eased
                    ox, oy, od = anim["from"]
                    if motion == "move":
                        fx = ox + (car.x - ox) * eased
                        fy = oy + (car.y - oy) * eased
                    else:
                        start = CAR_ANGLES[od]
                        delta = (angle - start + 540) % 360 - 180
                        angle = start + delta * eased
            cx = PADDING + (fx + 0.5) * CELL
            cy = PADDING + (fy + 0.5) * CELL
            self._draw_car(
                cx, cy, angle, CAR_COLORS[index], motion, phase,
                blocked=self.maze.has_wall(car.x, car.y, car.direction),
            )
            self.canvas.create_text(
                cx, cy,                 text=str(index + 1),
                fill=COLORS["text"], font=("Segoe UI", 10, "bold"),
            )
        direction_names = ("North", "East", "South", "West")
        results = "\n".join(
            f"Car {i + 1}: {self._competition_result(i)}"
            for i in range(len(self.cars))
        )
        car_details = "\n\n".join(
            self._car_status(index) for index in range(len(self.cars))
        )
        scenario = (
            "full quadrant exploration, then centre"
            if self.mode.get() == "explore"
            else "shortest path to centre"
        )
        self.status.set(
            f"Four-car competition ({scenario}) — {self._car_type_display()}\n"
            f"Elapsed: {self._elapsed_seconds():.1f} s\n"
            f"{results}\n\n{car_details}"
        )
        self.sensor_text.set(
            f"Sensors\n"
            f"  front: {'WALL' if self._sensor(0) else 'open'}\n"
            f"  left:  {'WALL' if self._sensor(3) else 'open'}\n"
            f"  right: {'WALL' if self._sensor(1) else 'open'}"
        )
        self.telemetry.set(
            f"Position: Column {self.robot.x + 1}, Row {self.robot.y + 1} | "
            f"Facing: {direction_names[self.robot.direction]} | "
            f"elapsed={self._elapsed_seconds():.1f}s | "
            f"recording={'on' if self.recording else 'off'}"
        )

    def _car_status(self, index: int) -> str:
        car = self.cars[index]
        direction_names = ("North", "East", "South", "West")
        sensor_names = ("front", "left", "right")
        sensors = ", ".join(
            f"{name}={'WALL' if self._car_sensor(car, relative) else 'open'}"
            for name, relative in zip(sensor_names, (0, 3, 1))
        )
        finished = (
            "Finished Race" if self.finished[index] is not None else "Running"
        )
        return (
            f"Car {index + 1}: Column {car.x + 1}, Row {car.y + 1}; "
            f"Facing {direction_names[car.direction]}; {finished}\n"
            f"  Sensors: {sensors}\n"
            f"  Distance: {self.car_distance[index]} cells; "
            f"Turns: {self.car_turns[index]}"
        )

    def _elapsed_seconds(self) -> float:
        if self.race_started_at is None:
            return 0.0
        end = self.race_finished_at if self.race_finished_at is not None else time.monotonic()
        return max(0.0, end - self.race_started_at)

    def _competition_result(self, index: int) -> str:
        zone_x, zone_y = CAR_ZONES[index]
        quadrant = {(x, y) for y in zone_y for x in zone_x}
        start = CAR_STARTS[index]
        reachable = self._reachable_exploration_cells(
            (start.x, start.y), quadrant, set()
        )
        explored = len(self.car_visited[index] & quadrant)
        percentage = 100 * explored / max(1, len(reachable))
        goal = (
            f"Goal Reached In {self.car_goal_times[index]:.1f} s"
            if self.car_goal_times[index] is not None else "Not Reached"
        )
        return (
            f"{goal}; Explored {percentage:.0f}% ({explored} cells); "
            f"Distance {self.car_distance[index]} cells; "
            f"Turns {self.car_turns[index]}"
        )

    def _draw_car(
        self, cx: float, cy: float, angle: float, color: str,
        motion: str = "idle", phase: float = 0.0, blocked: bool = False,
    ) -> None:
        def point(lateral: float, forward: float) -> tuple[float, float]:
            a = math.radians(angle)
            return (
                cx + forward * math.cos(a) - lateral * math.sin(a),
                cy + forward * math.sin(a) + lateral * math.cos(a),
            )

        board = [point(-9, -12), point(9, -12), point(10, 7),
                 point(0, 15), point(-10, 7)]
        self.canvas.create_polygon(
            *sum(board, ()), fill="#168f78", outline="#b8fff0", width=1
        )
        for lateral, forward in (
            (-10, -7), (10, -7), (-10, 7), (10, 7)
        ):
            wheel_x, wheel_y = point(lateral, forward)
            self.canvas.create_oval(
                wheel_x - 3, wheel_y - 4, wheel_x + 3, wheel_y + 4,
                fill="#080d18", outline="#64748b",
            )
        display = [point(-6, 8), point(6, 8), point(6, 1), point(-6, 1)]
        self.canvas.create_polygon(
            *sum(display, ()), fill="#08111f", outline="#67e8f9", width=1
        )
        board_center = point(0, -2)
        self.canvas.create_rectangle(
            board_center[0] - 4, board_center[1] - 5,
            board_center[0] + 4, board_center[1] + 5,
            fill="#334155", outline="#cbd5e1",
        )
        # Three range sensors on the nose - one centre, two angled. The
        # beams are drawn into the cell(s) ahead and animate so the
        # sensors visibly "work": they pulse forward while the car drives
        # and fan wider while it turns. When the car is at rest they stay
        # short and calm.
        if blocked:
            # A wall right in front: the beams stop at it and read solid,
            # the way a range sensor saturates on an obstacle.
            spread, reach, width, dash = 6, 5, 2, ()
        elif motion == "move":
            pulse = 0.5 + 0.5 * math.sin(time.monotonic() * 2 * math.pi / 0.5)
            spread, reach, width, dash = 6, 18 + int(10 * pulse), 2, (4, 3)
        elif motion == "turn":
            fan = phase
            spread, reach, width, dash = 8 + 4 * fan, 24, 2, (3, 2)
        else:
            spread, reach, width, dash = 6, 16, 1, (2, 3)
        for lateral in (-spread, 0.0, spread):
            x1, y1 = point(lateral, 13)
            x2, y2 = point(lateral * 2.2, 13 + reach)
            self.canvas.create_line(
                x1, y1, x2, y2, fill=color, width=width, dash=dash
            )


def main() -> None:
    root = tk.Tk()
    Simulator(root)
    root.mainloop()


if __name__ == "__main__":
    main()
