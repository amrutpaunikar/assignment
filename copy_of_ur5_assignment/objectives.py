from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt
import numpy as np
from isaacsim.core.utils.numpy.rotations import euler_angles_to_quats

from .config import AssignmentConfig
from .io_utils import write_csv
from .math_utils import normalize_quat, path_length
from .models import Objective2Result
from .robot_session import UR5RobotSession


class Objective1WorkspaceReachability:
    def __init__(self, session: UR5RobotSession, config: AssignmentConfig):
        self.session = session
        self.config = config

    def sample_workspace(
        self,
        n_points: int = 200,
        seed: int = 42,
        x_range=(0.30, 0.70),
        y_range=(-0.30, 0.30),
        z_range=(0.20, 0.35),
    ) -> List[Dict]:
        rng = np.random.default_rng(seed)
        xs = rng.uniform(x_range[0], x_range[1], size=n_points)
        ys = rng.uniform(y_range[0], y_range[1], size=n_points)
        zs = rng.uniform(z_range[0], z_range[1], size=n_points)
        quat = normalize_quat(np.asarray(euler_angles_to_quats(np.array([0.0, np.pi, 0.0]))))

        samples = []
        for idx, (x, y, z) in enumerate(zip(xs, ys, zs)):
            samples.append(
                {
                    "waypoint_idx": idx,
                    "position": np.array([x, y, z], dtype=float),
                    "quat": quat.copy(),
                }
            )
        return samples

    def plot_reachability(self, rows: List[Dict], save_path: Path) -> None:
        reachable = np.array([[r["x"], r["y"], r["z"]] for r in rows if r["Reachable"]], dtype=float)
        unreachable = np.array([[r["x"], r["y"], r["z"]] for r in rows if not r["Reachable"]], dtype=float)

        fig = plt.figure(figsize=(9, 7))
        ax = fig.add_subplot(111, projection="3d")
        if len(reachable) > 0:
            ax.scatter(reachable[:, 0], reachable[:, 1], reachable[:, 2], label="Reachable", s=20)
        if len(unreachable) > 0:
            ax.scatter(unreachable[:, 0], unreachable[:, 1], unreachable[:, 2], label="Unreachable", s=20)

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title("Workspace Reachability")
        ax.legend()
        plt.tight_layout()
        plt.savefig(save_path, dpi=180)
        plt.close(fig)

    def run(self, out_dir: Path) -> None:
        print("\nRunning Objective 1...")
        self.session.reset_arm_home()
        samples = self.sample_workspace()
        rows = []

        for item in samples:
            pos = item["position"]
            quat = item["quat"]
            success, q = self.session.apply_ik_target(pos, quat, settle_steps=self.config.settle_steps)

            if not success or q is None:
                reachable = False
                reason = "ik_failed"
                pos_err = np.nan
                ori_err = np.nan
            else:
                reachable, pos_err, ori_err, reason = self.session.validate_pose(pos, quat, q)

            rows.append(
                {
                    "Waypoint_idx": item["waypoint_idx"],
                    "x": float(pos[0]),
                    "y": float(pos[1]),
                    "z": float(pos[2]),
                    "qw": float(quat[0]),
                    "qx": float(quat[1]),
                    "qy": float(quat[2]),
                    "qz": float(quat[3]),
                    "Reachable": bool(reachable),
                    "Reason": reason,
                    "pos_error_m": pos_err,
                    "ori_error_rad": ori_err,
                }
            )

        csv_path = out_dir / "objective1_workspace_reachability.csv"
        write_csv(rows, csv_path)
        self.plot_reachability(rows, out_dir / "objective1_reachability_plot.png")
        print(f"Objective 1 CSV saved to: {csv_path}")


class Objective2ContinuousPath:
    def __init__(self, session: UR5RobotSession, config: AssignmentConfig):
        self.session = session
        self.config = config

    def make_continuous_path(self, n_points: int) -> Tuple[np.ndarray, np.ndarray]:
        theta = np.linspace(0.0, 2.0 * np.pi, n_points)
        center = np.array([0.52, 0.0, 0.33], dtype=float)
        radius_xy = 0.10
        z_amp = 0.06

        positions = np.zeros((n_points, 3), dtype=float)
        quats = np.zeros((n_points, 4), dtype=float)

        for i, t in enumerate(theta):
            x = center[0] + radius_xy * np.cos(t)
            y = center[1] + 0.14 * np.sin(t)
            z = center[2] + z_amp * np.sin(2.0 * t)
            positions[i] = np.array([x, y, z], dtype=float)

            yaw = 0.35 * np.sin(t)
            pitch = np.pi - 0.22 * np.cos(t)
            roll = 0.08 * np.sin(2.0 * t)
            quats[i] = normalize_quat(np.asarray(euler_angles_to_quats(np.array([roll, pitch, yaw]))))

        return positions, quats

    def plot_joint_positions(self, q_log: np.ndarray, save_path: Path) -> None:
        fig = plt.figure(figsize=(10, 6))
        for j in range(q_log.shape[1]):
            plt.plot(q_log[:, j], label=f"joint_{j + 1}")
        plt.xlabel("Waypoint index")
        plt.ylabel("Joint position (rad)")
        plt.title("Objective 2: Joint Position vs Waypoint")
        plt.legend(ncol=2)
        plt.tight_layout()
        plt.savefig(save_path, dpi=180)
        plt.close(fig)

    def plot_singularity_metric(self, sigma_log: np.ndarray, save_path: Path) -> None:
        fig = plt.figure(figsize=(9, 5))
        plt.plot(sigma_log)
        plt.axhline(0.02, linestyle="--", label="near-singular threshold")
        plt.xlabel("Waypoint index")
        plt.ylabel("Smallest singular value")
        plt.title("Objective 2: Singularity Metric")
        plt.legend()
        plt.tight_layout()
        plt.savefig(save_path, dpi=180)
        plt.close(fig)

    def run(self, out_dir: Path) -> Objective2Result:
        print("\nRunning Objective 2...")
        self.session.reset_arm_home()
        positions, quats = self.make_continuous_path(self.config.path_points)

        rows = []
        q_log = []
        sigma_log = []
        joint_jump_threshold = 0.75
        last_good_q = self.session.get_arm_joint_positions().copy()

        for i, (pos, quat) in enumerate(zip(positions, quats)):
            success, q = self.session.apply_ik_target(pos, quat, settle_steps=self.config.settle_steps)
            reachable = False
            reason = "ik_failed"
            sigma_min = np.nan

            if success and q is not None:
                jump = np.max(np.abs(q - last_good_q))
                jac = self.session.numerical_position_jacobian(q)
                singular_values = np.linalg.svd(jac, compute_uv=False)
                sigma_min = float(np.min(singular_values))
                near_singular = sigma_min < 0.02

                if jump > joint_jump_threshold:
                    reachable = False
                    reason = f"joint_jump_{jump:.3f}rad"
                    self.session.arm.set_joint_positions(last_good_q, joint_indices=self.session.arm_joint_indices)
                    self.session.step(2)
                    q_use = last_good_q.copy()
                elif near_singular:
                    reachable = True
                    reason = f"near_singular_sigma_{sigma_min:.4f}"
                    q_use = q.copy()
                    last_good_q = q.copy()
                else:
                    reachable = True
                    reason = "ok"
                    q_use = q.copy()
                    last_good_q = q.copy()
            else:
                q_use = last_good_q.copy()

            rows.append(
                {
                    "Waypoint_idx": i,
                    "x": float(pos[0]),
                    "y": float(pos[1]),
                    "z": float(pos[2]),
                    "qw": float(quat[0]),
                    "qx": float(quat[1]),
                    "qy": float(quat[2]),
                    "qz": float(quat[3]),
                    "Reachable": bool(reachable),
                    "Reason": reason,
                }
            )
            q_log.append(q_use)
            sigma_log.append(sigma_min)

        q_log = np.asarray(q_log, dtype=float)
        sigma_log = np.asarray(sigma_log, dtype=float)

        csv_path = out_dir / "objective2_continuous_path.csv"
        write_csv(rows, csv_path)
        self.plot_joint_positions(q_log, out_dir / "objective2_joint_positions.png")
        self.plot_singularity_metric(sigma_log, out_dir / "objective2_singularity_metric.png")
        print(f"Objective 2 CSV saved to: {csv_path}")

        return Objective2Result(positions=positions, quats=quats, q_log=q_log, sigma_log=sigma_log)


class Objective3TimeParameterization:
    def __init__(self, session: UR5RobotSession):
        self.session = session

    def time_parameterize(
        self, positions: np.ndarray, q_log: np.ndarray, sigma_log: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
        acc_limits = np.full_like(self.session.vel_limits, 1.5)
        n = len(q_log)
        dt = np.zeros(n - 1, dtype=float)
        prev_vel = np.zeros(q_log.shape[1], dtype=float)

        for i in range(n - 1):
            dq = q_log[i + 1] - q_log[i]
            dt_vel = float(np.max(np.abs(dq) / np.maximum(self.session.vel_limits, 1e-3)))
            dt_vel = max(dt_vel, 0.03)

            dt_i = dt_vel
            for _ in range(20):
                vel_i = dq / dt_i
                acc_i = (vel_i - prev_vel) / dt_i
                if np.all(np.abs(acc_i) <= acc_limits + 1e-6):
                    prev_vel = vel_i
                    break
                dt_i *= 1.15
            else:
                prev_vel = dq / dt_i

            if i < len(sigma_log) and np.isfinite(sigma_log[i]) and sigma_log[i] < 0.03:
                dt_i *= 1.4

            dt[i] = dt_i

        t = np.concatenate([[0.0], np.cumsum(dt)])
        qd = np.gradient(q_log, t, axis=0)
        qdd = np.gradient(qd, t, axis=0)
        tcp_speed = np.gradient(path_length(positions), t)
        target_speed_mm_s = tcp_speed * 1000.0
        max_feasible_tcp_speed_mm_s = float(np.nanmax(target_speed_mm_s))
        return t, qd, qdd, tcp_speed, target_speed_mm_s, max_feasible_tcp_speed_mm_s

    def save_objective_3_csv(self, positions: np.ndarray, quats: np.ndarray, target_speed_mm_s: np.ndarray, out_dir: Path):
        rows = []
        for i, (pos, quat, speed_mm_s) in enumerate(zip(positions, quats, target_speed_mm_s)):
            rows.append(
                {
                    "Waypoint_idx": i,
                    "x": float(pos[0]),
                    "y": float(pos[1]),
                    "z": float(pos[2]),
                    "qw": float(quat[0]),
                    "qx": float(quat[1]),
                    "qy": float(quat[2]),
                    "qz": float(quat[3]),
                    "target_linear_speed_mm_s": float(speed_mm_s),
                }
            )
        csv_path = out_dir / "objective3_time_parameterized_path.csv"
        write_csv(rows, csv_path)
        print(f"Objective 3 CSV saved to: {csv_path}")

    def plot_joint_kinematics(self, t: np.ndarray, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray, out_dir: Path):
        fig = plt.figure(figsize=(10, 6))
        for j in range(q.shape[1]):
            plt.plot(t, q[:, j], label=f"joint_{j + 1}")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (rad)")
        plt.title("Objective 3: Joint Position vs Time")
        plt.legend(ncol=2)
        plt.tight_layout()
        plt.savefig(out_dir / "objective3_joint_position.png", dpi=180)
        plt.close(fig)

        fig = plt.figure(figsize=(10, 6))
        for j in range(qd.shape[1]):
            plt.plot(t, qd[:, j], label=f"joint_{j + 1}")
        plt.xlabel("Time (s)")
        plt.ylabel("Velocity (rad/s)")
        plt.title("Objective 3: Joint Velocity vs Time")
        plt.legend(ncol=2)
        plt.tight_layout()
        plt.savefig(out_dir / "objective3_joint_velocity.png", dpi=180)
        plt.close(fig)

        fig = plt.figure(figsize=(10, 6))
        for j in range(qdd.shape[1]):
            plt.plot(t, qdd[:, j], label=f"joint_{j + 1}")
        plt.xlabel("Time (s)")
        plt.ylabel("Acceleration (rad/s^2)")
        plt.title("Objective 3: Joint Acceleration vs Time")
        plt.legend(ncol=2)
        plt.tight_layout()
        plt.savefig(out_dir / "objective3_joint_acceleration.png", dpi=180)
        plt.close(fig)

    def plot_tcp_speed(self, t: np.ndarray, tcp_speed: np.ndarray, out_dir: Path):
        fig = plt.figure(figsize=(9, 5))
        plt.plot(t, tcp_speed * 1000.0)
        plt.xlabel("Time (s)")
        plt.ylabel("TCP linear speed (mm/s)")
        plt.title("Objective 3: TCP Speed Along Path")
        plt.tight_layout()
        plt.savefig(out_dir / "objective3_tcp_speed.png", dpi=180)
        plt.close(fig)

    def run(self, out_dir: Path, result2: Objective2Result) -> np.ndarray:
        print("\nRunning Objective 3...")
        t, qd, qdd, tcp_speed, target_speed_mm_s, max_tcp_mm_s = self.time_parameterize(
            result2.positions, result2.q_log, result2.sigma_log
        )

        self.save_objective_3_csv(result2.positions, result2.quats, target_speed_mm_s, out_dir)
        self.plot_joint_kinematics(t, result2.q_log, qd, qdd, out_dir)
        self.plot_tcp_speed(t, tcp_speed, out_dir)

        stats_path = out_dir / "objective3_summary.txt"
        with open(stats_path, "w") as f:
            f.write(f"Fastest feasible TCP speed estimate: {max_tcp_mm_s:.2f} mm/s\n")
            f.write(f"Total trajectory time: {t[-1]:.2f} s\n")
            f.write(f"Velocity limits (rad/s): {self.session.vel_limits.tolist()}\n")

        print(f"Fastest feasible TCP speed estimate: {max_tcp_mm_s:.2f} mm/s")
        return t
