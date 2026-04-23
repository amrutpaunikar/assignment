# assignment

UR5-based robotics assignment implementations for Isaac Sim.

This repo now includes all three assignment variants from `C:\isaacsim\scripts`:

- `ur5_assignment` (base modular version)
- `copy_of_ur5_assignment` (copied variant)
- `ur5_assignment_plot_shown` (variant with plotted target markers and gripper-touch visualization in Objective 1)

## Setup

1. Use Isaac Sim 5.1 on Windows.
2. Ensure UR5 USD exists at:
   - `C:\isaacsim\assets\Isaac\5.1\Isaac\Robots\UniversalRobots\ur5\ur5.usd`
3. Optional props are read from:
   - `C:\isaacsim\assets\Isaac\5.1\Isaac\Props\YCB\Axis_Aligned_Physics\`
4. Run commands from:
   - `C:\isaacsim`

## File Structure

```text
assignment/                            <- this repository
  config.py                            <- base implementation config
  robot_session.py                     <- base implementation robot/session logic
  objectives.py                        <- base implementation objectives
  pipeline.py                          <- base implementation pipeline
  ...
  copy_of_ur5_assignment/              <- copied variant
    config.py
    robot_session.py
    objectives.py
    pipeline.py
    ...
  ur5_assignment_plot_shown/           <- plotted-target visualization variant
    config.py
    robot_session.py
    objectives.py
    pipeline.py
    ...
```

Local launcher scripts remain in `C:\isaacsim\scripts`:

- `isaac_sim_assignment_windows_ur5.py`
- `isaac_sim_assignment_windows_ur5_copy.py`
- `isaac_sim_assignment_windows_ur5_plot_shown.py`

## Commands (Without Props)

Run all commands from `C:\isaacsim`.

Base assignment:

```powershell
python.bat scripts\isaac_sim_assignment_windows_ur5.py --no_props
```

Copy assignment:

```powershell
python.bat scripts\isaac_sim_assignment_windows_ur5_copy.py --no_props
```

Plot-shown assignment:

```powershell
python.bat scripts\isaac_sim_assignment_windows_ur5_plot_shown.py --no_props
```

## Default Output Locations

- `ur5_assignment`:
  - `outputs\ur5_assignment\outputs`
- `copy_of_ur5_assignment`:
  - `outputs\copy_of_ur5_assignment\outputs`
- `ur5_assignment_plot_shown`:
  - `outputs\ur5_assignment_plot_shown\outputs`

## Output Files (All Three Variants)

Each variant generates these files in its own default output folder:

- `objective1_workspace_reachability.csv`
- `objective1_reachability_plot.png`
- `objective2_continuous_path.csv`
- `objective2_joint_positions.png`
- `objective2_singularity_metric.png`
- `objective3_time_parameterized_path.csv`
- `objective3_joint_position.png`
- `objective3_joint_velocity.png`
- `objective3_joint_acceleration.png`
- `objective3_tcp_speed.png`
- `objective3_summary.txt`

Note for `ur5_assignment_plot_shown`:



- If a CSV is open/locked (for example in Excel), it writes a timestamped fallback CSV and continues.

Outputs of the assignment present in this Drive link - https://drive.google.com/drive/folders/1Qyv5txH2rYOB0w_4Jk9wTcHvPaeIiHuP?usp=sharing
