#98709 Shirebots — VEX V5 Code Repository & Resource Hub

Welcome to the official code and resource repository for **98709 Shirebots**! This is the shared hub for all sub-teams in the organization. Each team has their own dedicated folder for source code and autonomous routines, and everyone pulls from a shared odometry library at the root level.

All programming is done in **Visual Studio Code** with the **VEX V5 Python extension** connected to this remote repository.
This is also a place to put documentation items and resources that are pertinent to the robotics season.

---

##Repository Structure

```
98709-robotics/
│
├── 📂 lib/                            # **Shared library** — all teams use this
│   ├── odometry.py                    # Core odometry (position tracking, movement)
│   ├── pid.py                         # PID controller (used by all drivetrain code)
│   ├── drivetrain.py                  # Shared drivetrain helpers
│   └── utils.py                       # General-purpose utilities
│
├── 📂 teams/                          # Per-team code folders
│   │
│   ├── 📂 98709A/                     # ← Team A (rename as needed)
│   │   ├── 📂 src/
│   │   │   ├── robot_config.py        # Motor ports, sensor ports for THIS robot
│   │   │   ├── driver_control.py      # Driver control code
│   │   │   └── sensors.py             # Sensor setup/calibration for THIS robot
│   │   ├── 📂 autonomous/
│   │   │   ├── auto_red_left.py       # Imports from lib/odometry.py
│   │   │   ├── auto_red_right.py
│   │   │   ├── auto_blue_left.py
│   │   │   └── auto_blue_right.py
│   │   └── 📂 notes/
│   │       └── port_map.md            # Wiring diagram notes for this robot
│   │
│   ├── 📂 98709B/                     # ← Team B (rename as needed)
│   │   ├── 📂 src/
│   │   │   ├── robot_config.py
│   │   │   ├── driver_control.py
│   │   │   └── sensors.py
│   │   ├── 📂 autonomous/
│   │   │   ├── auto_red_left.py
│   │   │   ├── auto_red_right.py
│   │   │   ├── auto_blue_left.py
│   │   │   └── auto_blue_right.py
│   │   └── 📂 notes/
│   │       └── port_map.md
│   │
│   └── 📂 98709C/                     # ← Team C (rename as needed)
│       ├── 📂 src/
│       │   ├── robot_config.py
│       │   ├── driver_control.py
│       │   └── sensors.py
│       ├── 📂 autonomous/
│       │   ├── auto_red_left.py
│       │   └── auto_blue_left.py
│       └── 📂 notes/
│           └── port_map.md
│
├── 📂 docs/                           # Reference documents for all teams
│   ├── game_manual.pdf
│   ├── field_layout.pdf
│   └── VEX_V5_Python_API.pdf
│
├── 📂 images/                         # Shared images and diagrams
│   ├── 📂 field/                      # Field maps, autonomous path diagrams
│   ├── 📂 robots/                     # Robot photos per team
│   └── 📂 wiring/                     # Motor/sensor wiring diagrams
│
├── .gitignore
└── README.md
```

> **Rule of thumb:** If code is specific to one robot (port numbers, motor directions, auton paths) → it goes in `teams/98709X/`. If it could be useful to any team → it goes in `lib/`.

---

## How the Shared Library Works

Each team's autonomous routines import directly from the shared `lib/` folder at the repo root. This means odometry, PID, and drivetrain code only needs to be written and fixed **once** — all teams benefit automatically.

### Example — how an autonomous file imports from `lib/`

```python
# teams/98709A/autonomous/auto_red_left.py

import sys, os

# Add the repo root to the path so Python can find lib/
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

from lib.odometry import Odometry
from lib.pid import PIDController
from teams._98709A.src.robot_config import motor_left, motor_right, inertial_sensor

# Set up odometry with THIS robot's hardware
odom = Odometry(motor_left, motor_right, inertial_sensor, wheel_diameter=3.25, track_width=12.0)
pid  = PIDController(kP=0.8, kI=0.0, kD=0.1)

def run():
    """Red left autonomous — scores preload and grabs one ring."""
    odom.reset()
    odom.drive_to(x=600, y=0)       # Drive forward 600 mm
    odom.turn_to(heading=90)         # Turn to face goal
    odom.drive_to(x=600, y=400)     # Drive to scoring position

run()
```

### Example — the shared `lib/odometry.py` skeleton

```python
# lib/odometry.py
# Shared across ALL 98709 teams — edit carefully, changes affect everyone!

from vex import *

class Odometry:
    def __init__(self, motor_left, motor_right, inertial, wheel_diameter, track_width):
        self.left    = motor_left
        self.right   = motor_right
        self.imu     = inertial
        self.wheel_d = wheel_diameter
        self.track_w = track_width
        self.x = 0.0
        self.y = 0.0

    def reset(self):
        """Reset position tracking to the origin."""
        self.x = 0.0
        self.y = 0.0
        self.left.reset_position()
        self.right.reset_position()
        self.imu.reset_heading()

    def drive_to(self, x: float, y: float, speed: int = 70):
        """Drive to an (x, y) field position in millimetres from the reset origin."""
        # TODO: implement full odometry tracking
        pass

    def turn_to(self, heading: float, speed: int = 50):
        """Turn to an absolute field heading in degrees (0 = forward)."""
        # TODO: implement PID-corrected turn using inertial sensor
        pass
```

> ⚠️ **Before editing anything in `lib/`** — give all team leads a heads-up. An issue here breaks every team's autonomous. Always work on a branch and open a Pull Request (see branching section below).

---

## Setup Guide — Getting Started (Read This First!)

### Step 1 — Install required tools

Make sure you have all three before anything else:

- [**Git**](https://git-scm.com/downloads) — the version control tool this repo runs on
- [**Visual Studio Code**](https://code.visualstudio.com/) — our shared code editor
- **VEX V5 Python Extension** — inside VS Code: `Ctrl+Shift+X` → search `VEX Robotics` → Install

### Step 2 — Clone the repository

1. Open VS Code
2. Press `Ctrl+Shift+P` → type `Git: Clone` → select it
3. Paste the repo URL: `https://github.com/[your-org]/98709-robotics.git`
4. Choose a folder to save it (e.g. `Documents/Robotics`)
5. Click **Open** when VS Code asks

### Step 3 — Set your Git identity (first time only)

Open the VS Code terminal (`Ctrl+\``) and run:

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@school.edu"
```

This tags your commits so the team can see who changed what.

---

## 🔄 Daily Workflow

Follow this **every time** you sit down to code — no skipping steps.

```bash
# 1. Always pull first — get the latest changes from GitHub
git pull

# 2. Write your code...

# 3. Check what files you've changed
git status

# 4. Stage your changes (your team's files only!)
git add teams/98709A/autonomous/auto_red_left.py
# or stage everything you changed:
git add .

# 5. Commit with a clear message — start with your team number
git commit -m "98709A: Add red left auto with vision alignment"

# 6. Push to GitHub
git push
```

**Golden rule: `git pull` before you start. `git push` when you're done.**

---

## 🤝 Collaboration Rules

- **Stay in your team's folder.** Don't edit another team's files without asking.
- **Never edit `lib/` alone.** Talk to all team leads first — it affects every robot.
- **Always pull before pushing.** Skipping this is how merge conflicts happen.
- **No broken code on `main`.** If it doesn't run on the brain, don't push it.
- **Write clear commit messages.** Start with your team: `98709B: fix turn overshoot`
- **Comment your code** — especially anything in `lib/`.

---

## 🧠 Code Conventions

```python
# ✅ Good — descriptive names, typed arguments, docstring
DRIVE_SPEED  = 75       # percent — tuned for competition tiles
TURN_TIMEOUT = 2000     # ms — prevents infinite loop on failed turn

def score_preload(odom: Odometry) -> None:
    """Drive to goal and score the preloaded ring."""
    odom.drive_to(x=500, y=0)
    odom.turn_to(heading=45)
```

```python
# ❌ Bad — unclear names, no comments, no types
def f(o):
    o.drive_to(500, 0)
    o.turn_to(45)
```

- **snake_case** for variables and functions (`auto_red_left`, `drive_to`)
- **UPPER_CASE** for constants (`MAX_SPEED`, `WHEEL_DIAMETER`)
- **Docstrings** on every function
- **Type hints** where possible (`speed: int = 70`)
- Prefix every commit message with your team number

---

## 🚦 Branching — Required for `lib/` Changes

If you're experimenting or modifying the shared `lib/` folder, always work on a branch:

```bash
# Create and switch to a new branch
git checkout -b 98709A/vision-auto-tuning

# ... make changes and commit as normal ...

# Push your branch (not main)
git push -u origin 98709A/vision-auto-tuning
```

Then open a **Pull Request** on GitHub so all leads can review before it merges. This is **required** for any changes to `lib/` — never push `lib/` edits directly to `main`.

---

## 🤖 VEX V5 Quick Reference

### Shared `lib/` files

| File | Purpose |
|------|---------|
| `lib/odometry.py` | Position tracking — `drive_to()`, `turn_to()` |
| `lib/pid.py` | PID controller class used internally by odometry |
| `lib/drivetrain.py` | Low-level motor helpers (arcade drive, tank drive) |
| `lib/utils.py` | Unit conversions, clamping, timing helpers |

### Per-team files

| File | Purpose |
|------|---------|
| `src/robot_config.py` | Motor/sensor port numbers for this specific robot |
| `src/driver_control.py` | Controller bindings and driver period loop |
| `src/sensors.py` | Sensor init, calibration, and helper wrappers |
| `autonomous/auto_*.py` | Individual auton routines — import from `lib/` |

---

## ❓ Getting Help

- **VEX Python API docs** → [api.vex.com](https://api.vex.com/)
- **VEX community forum** → [vexforum.com](https://www.vexforum.com/)
- **Git beginner guide** → [rogerdudler.github.io/git-guide](https://rogerdudler.github.io/git-guide/)
- **Stuck on a bug?** → Post in the team chat with your error message and what you've already tried

---

## 👥 Teams & Contacts

| Team | Lead Programmer | GitHub |
|------|----------------|--------|
| 98709A | [Name] | @username |
| 98709B | [Name] | @username |
| 98709C | [Name] | @username |

---

*Last updated: April 2026 — 98709 Robotics*
