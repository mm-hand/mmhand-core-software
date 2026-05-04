# Tactile Sim Visualizer

Standalone tactile visualization for development without physical tactile sensors.
This folder does not modify existing desktop control code.

## What it shows

- Dot-matrix animation
- 5 fingers x 3 sensors per finger
  - tip: 5x5
  - pad1: 4x13
  - pad2: 4x13
- Time-varying tactile values with smooth playback

## Run

```bash
python tactile_sim_visualizer/simulate_tactile_dot_matrix.py
```

Optional:

```bash
python tactile_sim_visualizer/simulate_tactile_dot_matrix.py --fps 20 --axis z --seed 20260424
```

Recommended high-contrast run:

```bash
python tactile_sim_visualizer/simulate_tactile_dot_matrix.py --axis z --fps 24 --boost 3.0 --speed 2.2
```

Keys:

- `space`: pause/resume

## Notes

- Shape and generation style are aligned with the desktop tactile module conventions.
- This is a simulator-only preview path for UI/algorithm iteration.
