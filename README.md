<div align="center">

# Micromouse Car Maze Simulator

**A four-car competitive Micromouse contest, simulated on a 16 × 16 maze — in your browser or on your desktop.**

[English](README.md) | [中文](README.zh-CN.md)

[![Live Demo](https://img.shields.io/badge/▶-Play_Online-4dabf7)](https://code-pulse-dev.github.io/Micromouse-Car-Maze-Simulator/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10%2B-blue)](https://www.python.org/)
[![No Dependencies](https://img.shields.io/badge/Dependencies-None-success)](#quick-start)

</div>

---

Watch four autonomous cars start in the four corners of the same maze, each sweep its own
quadrant with flood-fill navigation, learn the walls cell by cell, and race down a private
corridor into the centre goals. Then flip to Shortest Path mode and watch them race on a
fully known maze. Load your own car code and see whether *your* strategy beats the built-in one.

Built as part of a Micromouse competition project,and used to validate navigation
strategies.

## Table of Contents

- [Live Demo](#live-demo)
- [Quick Start](#quick-start)
- [What You Will See](#what-you-will-see)
- [Usage Guide](#usage-guide)
  - [Simulation Modes](#simulation-modes)
  - [Controls](#controls)
  - [Reading the Flood Values](#reading-the-flood-values)
  - [Maze Tools](#maze-tools)
  - [Hardware Panel](#hardware-panel)
- [Car Code Adapters](#car-code-adapters)
- [How the Simulation Works](#how-the-simulation-works)
- [Project Structure](#project-structure)
- [Verification](#verification)
- [FAQ](#faq)
- [Contributing](#contributing)
- [License](#license)

## Live Demo

No installation needed — open the hosted version and press **Run**:

**https://code-pulse-dev.github.io/Micromouse-Car-Maze-Simulator/**

The browser version mirrors the desktop app: the same three-column layout (maze, control
column, tabbed Movement / Maze Tools / Status / Car Code / Hardware panel), the same
four-car flood-fill sweep and shortest-path race, maze editing, generation, analysis,
save and load, recordings with replay, **your own car code running in the browser**
(Python via Pyodide) and a **real-car USB panel** using the Web Serial API.

## Quick Start

**Option 1 — Play in the browser (nothing to install)**

1. Open the [Live Demo](#live-demo) link.
2. Keep the default **Explore mode** and press **Run**.
3. Watch the cars sweep; press **Reset** and switch to **Shortest Path mode** to see the race.

**Option 2 — Run the desktop app (Windows / Linux / macOS)**

Requirements: Python 3.10 or newer with Tkinter. No third-party packages.

```bash
git clone https://github.com/Code-Pulse-Dev/Micromouse-Car-Maze-Simulator.git
cd Micromouse-Car-Maze-Simulator
python maze_simulator.py
```

**Option 3 — Download without Git**

Download the repository as a ZIP from the green **Code** button, extract it, then
run `python maze_simulator.py` (or double-click `index.html` for the browser version).

## What You Will See

| Element | Meaning |
|---|---|
| Four coloured triangles | The cars. Red, blue, green and purple, one per corner. |
| Orange numbers in cells | The **flood value** of car 1 — its live estimate of how many moves remain to the goal. |
| Gold cells labelled GOAL | The four centre goal cells. |
| Dashed coloured lines | Each car's currently planned route to its target. |
| Walls | The real maze. Cars do not know it at the start — they discover it with simulated sensors. |

## Usage Guide

### Simulation Modes

**Explore mode** — the first phase of a real Micromouse contest. Each car:

1. Sweeps its own 8 × 8 quadrant, always driving toward the most distant unvisited area first.
2. Records every wall it senses into its private map (the true maze is never revealed directly).
3. When its quadrant is fully covered, walks its **reserved corridor** — a private lane that no
   other car may enter — down into its goal, and stops.
4. Reports *Goal Reached In N s*, explored %, distance travelled and turn count.

**Shortest Path mode** — the second phase of the contest. All cars receive the full maze at
the start, plan the shortest route to their goal, and race. This mode answers the question
*how fast can the car go once the maze is known?*

### Controls

| Input | Action |
|---|---|
| **Run / Pause** | Start or freeze the whole simulation. |
| **Step** | Advance exactly one decision — useful for teaching the algorithm one move at a time. |
| **Reset** | Put every car back to its corner with an empty map. |
| **Mode select** | Switch between Explore and Shortest Path (resets the run). |
| **Step time slider** | One step = one forward move or one 90° turn. 100–400 ms. |
| **Trails checkbox** | Draw each car's path history on the maze. |

### Reading the Flood Values

Flood fill computes, for every cell, the minimum number of forward moves needed to reach the
goal through **known-open** passages. Goal cells show 0; neighbours show 1; and so on outward.
Because the car's map grows while it drives, the numbers keep changing — early in a run they
are an optimistic guess over a nearly empty map; by the end they are the true shortest
distances of the discovered maze. Unreachable cells stay blank.

### Maze Tools

- **Edit walls** — click a cell edge to toggle a wall (both sides update consistently).
- **Save / load mazes** — JSON files in `saved_mazes/`.
- The shipped maze is the real competition maze, including the team's measured wall edits.

### Hardware Panel

The desktop app can pair with a real car over USB (COM port + baud rate). It performs the
handshake, shows the car's name, and reads the real battery voltage in any of these formats:

- `BAT:7.86V` (volts) · `BAT:87` (percent) · `{"voltage": 7.86}` (JSON, millivolts accepted)

Charging status is derived from the voltage: *Fully Charged* above 8.2 V, *Needs Charging*
below 6.8 V (2S lithium pack). Without a car connected it honestly reports *Not Connected*.

## Car Code Adapters

The desktop app can replace the built-in sweep logic with **your** controller, per car:

1. Pick a car slot in the Car Code panel.
2. Click *Upload Code Folder* and select your code folder.
3. The simulator matches it to an adapter in `car_codes/` — or to a `CarController.py`
   inside your folder — and runs it step by step.

**The adapter contract.** A `CarController` class receives a per-step state dictionary
(position, heading, sensor readings, private map) and returns one movement command:
`forward`, `left`, `right` or `turnback`. Use `car_template.py` as the starting point.

**Fair play is enforced by the simulator, not by your code.** Quadrant limits, reserved
corridors and collision rules apply to every adapter equally, so a buggy or cheating
controller cannot break the contest.

Two ready-made adapters ship with the project — faithful ports of the team's real cars:

| Adapter | Port of | Behaviour |
|---|---|---|
| `ESP32-S3 Micromouse Adapter` | the ESP32-S3 car's firmware strategy (ToF sensors, MPU6050 turns) | identical move sequence to the real car |
| `TQD Micromouse JS2 Adapter` | the TQD car's MicroPython strategy (right-hand search, contour-map navigation) | identical move sequence to the real car |

On the standard maze both adapters reproduce the built-in sweep move-for-move — verified by
regression scripts — so simulated results reflect what the real cars would do.

## How the Simulation Works

- **Maze model** — a 16 × 16 grid; every cell stores four wall bits (N/E/S/W). The outer
  border is always closed.
- **Competition geography** — four 8 × 8 quadrants, one start corner and one goal per car
  (`(0,0)→(7,7)`, `(15,0)→(8,7)`, `(0,15)→(7,8)`, `(15,15)→(8,8)`), plus four reserved
  corridors that keep the simultaneous run collision-free.
- **Decision loop (per car, per step)** — sense → write walls into the private map →
  flood-fill over the known map → choose the best neighbour (straight-ahead preferred on
  ties) → turn and advance.
- **Timing** — elapsed race time = steps × step time. Distance counts every forward move,
  revisits included; explored % counts unique cells visited within the quadrant (64 cells).

## Project Structure

```
Micromouse-Car-Maze-Simulator/
├── index.html                       browser version (served by GitHub Pages)
├── maze_simulator.py                desktop application: GUI, maze model, flood fill, sweep
├── car_template.py                  starting point for writing your own car adapter
├── car_codes/
│   ├── ESP32-S3 Micromouse Adapter/
│   │   └── CarController.py         adapter porting the ESP32-S3 car's strategy
│   └── TQD Micromouse JS2 Adapter/
│       └── CarController.py         adapter porting the TQD JS2 car's strategy
├── saved_mazes/
│   ├── Maze.json                    the real competition maze
│   └── real_maze_edits.json         measured wall corrections
└── README.md / README.zh-CN.md      this documentation (English / Chinese)
```

## Verification

The simulation logic is validated by headless regression runs that assert, on every change:

- Every car covers **64 / 64 cells (100 %)** of its quadrant in Explore mode.
- Every car finishes through its own corridor — no car ever enters another car's reserved corridor.
- Shortest Path runs reach the goals with the optimal route.
- The two car-code adapters reproduce the built-in sweep move-for-move on the standard maze.

## FAQ

**Why do the flood numbers keep changing?**
The car only knows the walls it has sensed. Its cost map is recomputed over the known map
every step, so it becomes more accurate — and shorter — as the maze is learned.

**Can two cars collide?**
No. During exploration every car is confined to its own quadrant and reserved corridor, so
the four simultaneous runs never share a cell.

**Does the browser version have the car-code adapters and hardware panel?**
Yes. Python car code runs in the browser through Pyodide (WebAssembly Python), so both
shipped adapters and your own `CarController.py` work on the hosted site. The hardware
panel uses the Web Serial API — supported in Chrome and Edge over https — to pair a real
car and read its battery; no other browser can reach a serial port by design.

**Is this tied to a specific maze?**
No. Generate, edit or load any 16 × 16 maze and the same logic applies.

## Contributing

Issues and pull requests are welcome. If you write a new car adapter, open a pull request
with your `CarController.py` — the regression scripts will check that it plays fair.

## License

Released under the [MIT License](LICENSE).
