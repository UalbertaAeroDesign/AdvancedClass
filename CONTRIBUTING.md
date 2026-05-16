# Contributing — UAlberta Aero Design Advanced Class

Welcome. This document is for new team members joining the Advanced Class
software/electrical side. Read this first so you don't waste your first week
on a wild goose chase.

---

## Repo layout

```
competition_2026/     scripts that run at competition (on the RPi or laptop)
simulation/           Gazebo + Webots SITL scripts (no real hardware)
detection/            AprilTag + YOLO + camera calibration
test_flights/         scripts used for real flight tests, organised by airframe
hardware/             motor mapping, payload STM32 code, payload chassis CAD
config/               flight controller parameter files, config templates
docs/                 setup guides, wiring diagrams, flight log template
scripts/              shell scripts for SITL setup, environment bootstrapping
```

When adding code, drop it in the directory whose purpose best matches —
don't pile new scripts at the repo root.

---

## First-time setup

### 1. Clone the repo

```bash
git clone https://github.com/UalbertaAeroDesign/AdvancedClass.git
cd AdvancedClass
```

### 2. Python environment

```bash
python -m venv .venv
source .venv/bin/activate           # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

The repo's `.gitignore` already excludes `.venv/` — keep your venv local.

### 3. ArduPilot SITL (for any sim work)

The ArduPilot source tree is **not** committed to this repo. Clone it
separately and add the autotest tools to your `PATH`:

```bash
cd ~                                # outside this repo
git clone --recursive https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-mac.sh    # or -ubuntu.sh
git checkout ArduPlane-4.5          # or whatever stable release is current
./waf configure --board sitl
./waf plane                         # or `./waf copter` for ArduCopter
```

Then add to `~/.zshrc` / `~/.bashrc`:

```bash
export PATH="$HOME/ardupilot/Tools/autotest:$PATH"
```

Verify:

```bash
which sim_vehicle.py                # should print a path
```

### 4. Real-hardware setup (RPi 5 connected to FC)

See [`docs/rpi5_setup.md`](docs/rpi5_setup.md) for the full sequence. Highlights:

- `picamera2` ships with Raspberry Pi OS — don't install via pip
- UART pins: GPIO 14 (TX), GPIO 15 (RX), shared GND with the H743
- MAVLink runs at 921600 baud on `/dev/ttyAMA0`

---

## Day-to-day workflow

### Running SITL with a quadplane

```bash
sim_vehicle.py -v ArduPlane -f quadplane --console --map
```

Open Mission Planner (or QGC) and connect on UDP `127.0.0.1:14550`.

### Running SITL with a quadcopter

```bash
sim_vehicle.py -v ArduCopter -f quad --console --map
```

### Uploading a mission

Edit `competition_2026/upload_mission.py` (DLZ lat/lon at the top), then:

```bash
python competition_2026/upload_mission.py                # over UDP (SITL)
python competition_2026/upload_mission.py --port /dev/ttyAMA0   # real hw
```

### Loading flight controller parameters

```bash
# In Mission Planner: Config → Full Parameter List → Load from file
# Pick from config/params/
```

---

## Branching / PR conventions

- `main` is the working branch; commit small, working changes directly when
  it's just you in the repo
- For larger features, open a **feature branch** named `feat/short-description`
- Keep commits focused — one logical change per commit
- Commit messages: imperative mood, short subject line, optional body
  - Good: `add NAV_DELAY to release sequence`
  - Bad: `updates`

Push, open a PR, get one other person on the team to look at it before
merging. Two eyes catches the obvious mistakes.

---

## Flight test discipline

**Every real test flight gets a log entry.** Copy
[`docs/flight_log_template.md`](docs/flight_log_template.md) into a new file
named `docs/flight_logs/YYYY-MM-DD_description.md` and fill it in *before*,
*during*, and *after* the flight. Future you will not remember which set of
params was loaded on a particular day. Past us has been burned by this.

---

## Adding a new script

Before writing a new script, search the repo first — there's a good chance
someone wrote something similar already. Common patterns:

- AprilTag detection → start from `detection/april_tag/ov9281_tag_detect.py`
- MAVLink connection + arming → start from
  `simulation/gazebo/tricopter/manual_takeoff_then_auto.py`
- Mission upload → `competition_2026/upload_mission.py`

When you add a new script:

- Put a docstring at the top explaining what it does and how to run it
- Configuration constants at the top of the file (not buried in main)
- Use `argparse` for command-line options if it has more than one flag

---

## Asking for help

- For ArduPilot-specific questions, the [ArduPilot forums](https://discuss.ardupilot.org/) are excellent
- For repo-specific questions, ask on the team Slack / Discord first, then
  open a GitHub issue if it's a code bug worth tracking

If you spend more than an hour stuck on something, *ask*. Time is precious
during competition season.
