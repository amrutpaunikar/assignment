# assignment

UR5 assignment module for Isaac Sim (modular OOP version).

## What This Contains

- UR5 robot session and scene setup
- Objective 1: workspace reachability
- Objective 2: continuous IK path generation
- Objective 3: time-parameterized trajectory + plots
- CSV/plot export helpers

## Required Paths

- UR5 USD is expected at:
  - `C:\isaacsim\assets\Isaac\5.1\Isaac\Robots\UniversalRobots\ur5\ur5.usd`
- Optional props are loaded from:
  - `C:\isaacsim\assets\Isaac\5.1\Isaac\Props\YCB\Axis_Aligned_Physics\`

## Run Commands

Run from Isaac Sim root:

```powershell
cd C:\isaacsim
python.bat scripts\isaac_sim_assignment_windows_ur5.py
```

Headless mode:

```powershell
cd C:\isaacsim
python.bat scripts\isaac_sim_assignment_windows_ur5.py --headless
```

Custom output folder:

```powershell
cd C:\isaacsim
python.bat scripts\isaac_sim_assignment_windows_ur5.py --out_dir outputs\ur5_assignment_run1
```

Disable props:

```powershell
cd C:\isaacsim
python.bat scripts\isaac_sim_assignment_windows_ur5.py --no_props
```

## Expected Outputs

By default, outputs are saved under `outputs\` and include:

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

## Recordings

- This module does not auto-record video.
- Record from Isaac Sim GUI using Recorder.
- Your current frame capture path is:
  - `C:\Users\prati\Documents\Kit\shared\screenshots\`

## GitHub Push Commands

If you want to initialize and push this folder to GitHub:

```powershell
echo "# assignment" >> README.md
git init
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/amrutpaunikar/assignment.git
git push -u origin main
```

For this project (recommended, includes all code files):

```powershell
git init
git add .
git commit -m "first commit"
git branch -M main
git remote add origin https://github.com/amrutpaunikar/assignment.git
git push -u origin main
```
