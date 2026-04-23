import argparse
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class AssignmentConfig:
    headless: bool
    out_dir: Path
    physics_dt: float
    render_dt: float
    settle_steps: int
    path_points: int
    playback_loops: int
    spawn_props: bool
    ur5_usd_path: str = r"C:\isaacsim\assets\Isaac\5.1\Isaac\Robots\UniversalRobots\ur5\ur5.usd"

    @property
    def simulation_app_config(self) -> dict:
        return {
            "headless": self.headless,
            "renderer": "RayTracedLighting",
            "width": 1280,
            "height": 720,
        }


def parse_assignment_config() -> AssignmentConfig:
    parser = argparse.ArgumentParser(description="Isaac Sim assignment solution (plot shown) on Windows using UR5")
    parser.add_argument("--headless", action="store_true", help="Run without GUI")
    parser.add_argument(
        "--out_dir",
        type=str,
        default="outputs/ur5_assignment_plot_shown/outputs",
        help="Output folder",
    )
    parser.add_argument("--physics_dt", type=float, default=1.0 / 60.0, help="Physics timestep")
    parser.add_argument("--render_dt", type=float, default=1.0 / 60.0, help="Render timestep")
    parser.add_argument("--settle_steps", type=int, default=12, help="Simulation steps after IK action")
    parser.add_argument("--path_points", type=int, default=140, help="Points in continuous Cartesian path")
    parser.add_argument("--playback_loops", type=int, default=2, help="Replay loops for final demo")
    parser.add_argument("--no_props", action="store_true", help="Disable loading props/material assets into the scene")
    args, _ = parser.parse_known_args()

    return AssignmentConfig(
        headless=args.headless,
        out_dir=Path(args.out_dir),
        physics_dt=args.physics_dt,
        render_dt=args.render_dt,
        settle_steps=args.settle_steps,
        path_points=args.path_points,
        playback_loops=args.playback_loops,
        spawn_props=not args.no_props,
    )
