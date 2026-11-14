# Micromouse Car Maze Simulator

A standalone Python/Tkinter application that simulates a **four-car
competitive Micromouse contest** on a 16 x 16 maze. Four cars start in the
four corners, each sweeps its own quadrant, learns the maze wall by wall and
races to the centre — either with the built-in flood-fill sweep logic or with
your own car code loaded as an adapter.

Built as part of a university Micromouse competition project, and used to
test navigation strategies before risking the real robots.

## Play online

A browser version is included: open `index.html` via GitHub Pages
(`https://code-pulse-dev.github.io/Micromouse-Car-Maze-Simulator/`) or simply
download the repo and double-click the file - no install, no dependencies.
It runs the same 16 x 16 maze with the same flood-fill sweep, reserved
corridors and shortest-path race as the desktop app.

## Features

- **Four-car competition** — per-car quadrants, private reserved corridors to
  the centre goals, collision-free simultaneous runs
- **Flood-fill navigation** — each car builds its own map from simulated
  sensors; the displayed flood values are the live "thoughts" of car 1
- **Explore mode** (sweep the quadrant, then walk the reserved corridor into
  the goal) and **Shortest Path mode** (race on a fully known maze)
- **Car code adapters** — load a car code folder and the simulator runs your
  controller instead of the built-in logic (see `car_template.py`)
- **Two ready adapters** — ports of the real ESP32-S3 car (ToF sensors) and
  TQD-Micromouse JS2 (linear IR) strategies
- **Maze tools** — edit walls by clicking, generate mazes, save/load (JSON)
- **Hardware panel** — pair a real car over USB; it reports its name and real
  battery voltage
- **Live visuals** — smooth glide/turn animation, animated sensor beams,
  dashed route to target, per-car statistics (explored %, distance, turns,
  goal time)

## Running the desktop app

Python 3.10+ with Tkinter — no third-party packages.

```
python maze_simulator.py
```

## Loading your own car code (desktop app)

1. Pick a car slot in the Car Code panel.
2. Click *Upload Code Folder* and select your code folder.
3. The simulator matches it to an adapter in `car_codes/` (or a
   `CarController.py` inside your folder) and runs it step by step.

The competition rules (quadrant limits, reserved corridors, collision
checking) are enforced by the simulator, so any adapter plays fair.

## Project structure

```
maze_simulator.py            application: GUI, maze model, flood fill, sweep logic
index.html                   browser version (works on GitHub Pages)
car_template.py              template for writing your own car adapter
car_codes/
  ESP32-S3 Micromouse Adapter/CarController.py
  TQD Micromouse JS2 Adapter/CarController.py
saved_mazes/                 saved maze files (JSON)
```

## Licence

MIT — see [LICENSE](LICENSE).
