# Lap Time Simulation Engine

A C++ quasi-steady-state lap simulator for race cars and road cars.

Give it a track and a vehicle definition, and it will:
- estimate the fastest achievable lap time
- generate per-segment telemetry
- export a GGV map
- write outputs automatically to `outputs/`

The current version is self-contained:
- no `JsonCpp`
- no `Eigen`
- direct JSON parsing is built into the repo
- `build.sh` can fall back to a direct `g++` build when `cmake` is not installed

## What It Models

- aerodynamic drag and downforce
- tire grip with load sensitivity
- longitudinal and lateral force sharing
- engine torque curve, gearing, final drive, and shift time
- forward/backward speed solving around a closed lap
- telemetry export in CSV and optional JSON

This is still a quasi-steady-state simulator, not a full transient vehicle model. It does not model things like:
- suspension kinematics
- tire temperature or wear
- fuel burn
- DRS
- ERS deployment strategy
- differential tuning
- detailed aero platform sensitivity

## Quick Start

### Linux / macOS

```bash
chmod +x build.sh
./build.sh

./build/lap_sim examples/montreal.csv examples/f1_2025.json
```

### Windows

```bat
build.bat
build\Release\lap_sim.exe examples\montreal.csv examples\f1_2025.json
```

## CLI

```bash
./build/lap_sim <track_csv_or_json> <vehicle_json> [options]
```

Options:

- `--csv <file>` write telemetry CSV to a specific path
- `--json <file>` write telemetry JSON to a specific path
- `--ggv <file>` write GGV CSV to a specific path
- `--iterations <N>` solver iteration cap, default `10`
- `--tolerance <T>` convergence tolerance, default `0.001`
- `--help` print usage

If you do not provide output paths, the simulator still writes:
- telemetry CSV to `outputs/<car>-<track>-<mm_ss>-VSIM.csv`
- GGV CSV to `outputs/<car>-<track>-<mm_ss>-VSIM-GGV.csv`

## Included Examples

Vehicle presets in `examples/`:

- `f1_2025.json` balanced 2025 F1 baseline
- `f1_2025_normal.json`
- `f1_2025_monza.json`
- `f1_2025_monaco.json`
- `f1_2024.json` balanced 2024 F1 baseline
- `f1_2024_normal.json`
- `f1_2024_monza.json`
- `f1_2024_monaco.json`
- `fsae_road_course.json`
- `honda_civic_si_2025.json`

Track files in `examples/`:

- `montreal.csv`
- `Zandvoort.csv`
- `Monza.csv`
- `Shanghai.csv`

Useful runs:

```bash
./build/lap_sim examples/Monza.csv examples/f1_2025_monza.json
./build/lap_sim examples/Zandvoort.csv examples/f1_2025_monaco.json
./build/lap_sim examples/montreal.csv examples/f1_2024.json
./build/lap_sim examples/montreal.csv examples/fsae_road_course.json
./build/lap_sim examples/montreal.csv examples/honda_civic_si_2025.json
```

## Vehicle File Format

Vehicle definitions are JSON files.

Example:

```json
{
  "name": "F1_2025_Normal",
  "mass": {
    "mass": 800.0,
    "cog_height": 0.25,
    "wheelbase": 3.60,
    "weight_distribution": 0.46
  },
  "aerodynamics": {
    "Cl": -4.2,
    "Cd": 0.95,
    "frontal_area": 1.42,
    "air_density": 1.225
  },
  "tire": {
    "mu_x": 1.95,
    "mu_y": 2.34,
    "load_sensitivity": 0.75,
    "tire_radius": 0.33
  },
  "powertrain": {
    "engine_torque_curve": {
      "5000": 370,
      "6000": 400,
      "7000": 430
    },
    "gear_ratios": [10.8, 8.9, 7.4, 6.4, 5.7, 5.1, 4.5, 3.9],
    "final_drive": 1.38,
    "efficiency": 0.98,
    "max_rpm": 15000,
    "min_rpm": 5000,
    "shift_time": 0.035
  },
  "brake": {
    "max_brake_force": 32000,
    "brake_bias": 0.62
  }
}
```

Field notes:

- `mass.mass` total vehicle mass in kg
- `mass.weight_distribution` front axle fraction from `0.0` to `1.0`
- `aerodynamics.Cl` should be negative for downforce-producing cars
- `tire.mu_x` and `tire.mu_y` are longitudinal and lateral grip coefficients
- `powertrain.engine_torque_curve` maps RPM to torque in Nm
- `powertrain.gear_ratios` must be listed from shortest gear to tallest gear
- `powertrain.shift_time` is optional
- `brake.brake_bias` is the front brake fraction from `0.0` to `1.0`

## Track File Format

The simulator supports:
- TUMFTM-style CSV tracks
- JSON track input through the parser

The CSV format used by the bundled tracks is:

```csv
# x_m,y_m,w_tr_right_m,w_tr_left_m
0.123,-0.739,5.388,5.699
1.227,-5.613,5.352,5.669
2.331,-10.486,5.316,5.640
```

Columns:

- `x_m` centerline X coordinate
- `y_m` centerline Y coordinate
- `w_tr_right_m` track half-width to the right
- `w_tr_left_m` track half-width to the left

The solver preprocesses the centerline into arc length, heading, and curvature before solving.

## Output Data

The CSV telemetry includes:

- timestamp
- arc length
- XYZ position
- lateral offset from the working line
- speed in m/s and km/h
- longitudinal, lateral, and vertical acceleration
- longitudinal, lateral, and total g
- throttle and brake percentage
- steering angle
- gear and RPM
- engine torque and wheel force
- drag, downforce, longitudinal tire force, lateral tire force, and vertical load
- curvature, radius, and banking

The optional JSON export contains the same core telemetry grouped into:

- position
- velocity
- acceleration
- g-forces
- controls
- powertrain
- forces
- track

## Build Notes

### Linux / macOS

- if `cmake` is available, `build.sh` uses it
- otherwise `build.sh` compiles directly with `g++`

### Windows

- `build.bat` uses CMake and builds `build\Release\lap_sim.exe`

### Dependencies

Current direct dependencies are just a C++17-capable compiler and standard library support.

Typical toolchains:

- Linux: GCC or Clang
- macOS: Apple Clang via Xcode Command Line Tools
- Windows: MSVC via Visual Studio or Build Tools

## Repo Layout

```text
.
├── README.md
├── CMakeLists.txt
├── build.sh
├── build.bat
├── examples/
├── include/
│   ├── data/
│   ├── io/
│   ├── physics/
│   ├── solver/
│   └── telemetry/
├── src/
│   ├── data/
│   ├── io/
│   ├── physics/
│   ├── solver/
│   └── telemetry/
└── outputs/
```

## Implementation Summary

At a high level the solver does this:

1. load track and vehicle data
2. preprocess the track into a working path
3. estimate a bounded racing line inside the track widths
4. smooth curvature on the working path
5. compute cornering speed limits
6. sweep forward for acceleration limits
7. sweep backward for braking limits
8. iterate until lap time converges
9. generate detailed telemetry
10. export CSV, optional JSON, and GGV data

## Verification

The current codebase has been exercised with:

- `bash build.sh`
- `./build/lap_sim examples/Monza.csv examples/f1_2025_monza.json`
- `./build/lap_sim examples/Zandvoort.csv examples/f1_2025_monaco.json`
- `./build/lap_sim examples/montreal.csv examples/honda_civic_si_2025.json`

## Credits

This project is heavily inspired by the public research and tooling published by TUM Fast Technology Munich.

Useful references:

- TUMFTM laptime simulation: https://github.com/TUMFTM/laptime-simulation
- TUMFTM racetrack database: https://github.com/TUMFTM/racetrack-database
- Christ, Wischnewski, Heilmeier, Lohmann (2019): https://doi.org/10.1080/00423114.2019.1704804
- Heilmeier, Graf, Lienkamp (2018): https://doi.org/10.1109/ITSC.2018.8570012

The implementation in this repository is C++ and independent, but the overall workflow and track-format ideas are informed by that work.
