import os
from typing import Optional, Tuple

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.utils.numpy.rotations import rot_matrices_to_quats
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
    interface_config_loader,
)

from .config import AssignmentConfig
from .math_utils import quat_distance
from .scene_assets import SceneAssetComposer


class UR5RobotSession:
    def __init__(self, config: AssignmentConfig):
        self.config = config
        self.world = World(
            stage_units_in_meters=1.0,
            physics_dt=config.physics_dt,
            rendering_dt=config.render_dt,
        )
        self.arm: Optional[Robot] = None
        self.kin_solver = None
        self.art_kin_solver = None
        self.ee_frame = "tool0"

        self.num_arm_dofs = 6
        self.arm_joint_indices: Optional[np.ndarray] = None
        self.lower_limits: Optional[np.ndarray] = None
        self.upper_limits: Optional[np.ndarray] = None
        self.vel_limits: Optional[np.ndarray] = None
        self.scene_assets = SceneAssetComposer()

    def step(self, steps: int = 1) -> None:
        for _ in range(steps):
            self.world.step(render=not self.config.headless)

    def setup_scene(self) -> None:
        self.world.scene.clear()
        self.world.scene.add_default_ground_plane()
        if self.config.spawn_props:
            self.scene_assets.populate_props("/World/Props")

        if not os.path.exists(self.config.ur5_usd_path):
            raise FileNotFoundError(f"UR5 USD not found: {self.config.ur5_usd_path}")

        robot_root_prim_path = "/World/UR5"
        add_reference_to_stage(usd_path=self.config.ur5_usd_path, prim_path=robot_root_prim_path)

        robot_prim_path = (
            f"{robot_root_prim_path}/root_joint"
            if is_prim_path_valid(f"{robot_root_prim_path}/root_joint")
            else robot_root_prim_path
        )

        self.arm = Robot(prim_path=robot_prim_path, name="ur5")
        self.world.scene.add(self.arm)

        self.world.reset()
        self.arm.initialize()

        supported = interface_config_loader.get_supported_robots_with_lula_kinematics()
        if "UR5" in supported:
            kin_config = interface_config_loader.load_supported_lula_kinematics_solver_config("UR5")
            self.kin_solver = LulaKinematicsSolver(**kin_config)
        else:
            mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
            kinematics_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")
            self.kin_solver = LulaKinematicsSolver(
                robot_description_path=os.path.join(
                    kinematics_config_dir,
                    "universal_robots",
                    "ur5",
                    "rmpflow",
                    "ur5_robot_description.yaml",
                ),
                urdf_path=os.path.join(kinematics_config_dir, "universal_robots", "ur5", "ur5.urdf"),
            )

        available_frames = set(self.kin_solver.get_all_frame_names())
        for ee_candidate in ("tool0", "flange", "ee_link"):
            if ee_candidate in available_frames:
                self.ee_frame = ee_candidate
                break

        self.art_kin_solver = ArticulationKinematicsSolver(self.arm, self.kin_solver, self.ee_frame)

        names = self.arm.dof_names
        print("Loaded robot DOFs:", names)

        canonical_ur5_joints = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        if all(j in names for j in canonical_ur5_joints):
            arm_names = canonical_ur5_joints
        else:
            arm_names = [n for n in names if "finger" not in n.lower() and "gripper" not in n.lower()][
                : self.num_arm_dofs
            ]

        if len(arm_names) != self.num_arm_dofs:
            raise RuntimeError(
                f"Expected {self.num_arm_dofs} UR5 arm joints, but found {len(arm_names)}: {arm_names}"
            )

        self.arm_joint_indices = np.array([self.arm.get_dof_index(n) for n in arm_names], dtype=int)

        dof_props = self.arm.dof_properties
        self.lower_limits = np.asarray(dof_props["lower"], dtype=float)[self.arm_joint_indices]
        self.upper_limits = np.asarray(dof_props["upper"], dtype=float)[self.arm_joint_indices]
        self.vel_limits = np.array([3.14, 3.14, 3.14, 3.14, 3.14, 3.14], dtype=float)

        print("Using arm DOFs:", arm_names)
        print("Joint indices:", self.arm_joint_indices)
        print("Lower limits:", self.lower_limits)
        print("Upper limits:", self.upper_limits)
        print("Velocity limits:", self.vel_limits)
        print("EE frame:", self.ee_frame)

    def reset_arm_home(self) -> None:
        q_home = np.array([0.0, -np.pi / 2.0, np.pi / 2.0, -np.pi / 2.0, -np.pi / 2.0, 0.0], dtype=float)
        q_home = np.clip(q_home, self.lower_limits + 1e-3, self.upper_limits - 1e-3)
        self.arm.set_joint_positions(q_home, joint_indices=self.arm_joint_indices)
        self.step(20)

    def set_solver_base_pose(self) -> None:
        if hasattr(self.arm, "get_world_pose"):
            base_t, base_q = self.arm.get_world_pose()
        else:
            base_t_all, base_q_all = self.arm.get_world_poses()
            base_t = base_t_all[0]
            base_q = base_q_all[0]

        if hasattr(base_t, "detach"):
            base_t = base_t.detach().cpu().numpy()
        elif hasattr(base_t, "numpy"):
            base_t = base_t.numpy()

        if hasattr(base_q, "detach"):
            base_q = base_q.detach().cpu().numpy()
        elif hasattr(base_q, "numpy"):
            base_q = base_q.numpy()

        self.kin_solver.set_robot_base_pose(np.asarray(base_t, dtype=float), np.asarray(base_q, dtype=float))

    def get_arm_joint_positions(self) -> np.ndarray:
        q = np.asarray(self.arm.get_joint_positions(), dtype=float)
        return q[self.arm_joint_indices]

    def get_ee_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        ee_pos, ee_rot = self.art_kin_solver.compute_end_effector_pose()
        ee_pos = np.asarray(ee_pos, dtype=float)
        ee_rot = np.asarray(ee_rot, dtype=float)

        if ee_rot.shape == (3, 3):
            ee_quat = np.asarray(rot_matrices_to_quats(ee_rot), dtype=float)
        elif ee_rot.shape == (4,):
            ee_quat = ee_rot
        else:
            ee_quat = ee_rot.reshape(-1)[:4]

        return ee_pos, ee_quat

    def apply_ik_target(
        self, pos: np.ndarray, quat: np.ndarray, settle_steps: Optional[int] = None
    ) -> Tuple[bool, Optional[np.ndarray]]:
        settle_steps = self.config.settle_steps if settle_steps is None else settle_steps

        self.set_solver_base_pose()
        action, success = self.art_kin_solver.compute_inverse_kinematics(pos, quat)
        if not success:
            return False, None

        self.arm.apply_action(action)
        self.step(settle_steps)
        return True, self.get_arm_joint_positions()

    def validate_pose(self, target_pos: np.ndarray, target_quat: np.ndarray, q_sol: np.ndarray):
        in_limits = bool(np.all(q_sol >= self.lower_limits - 1e-4) and np.all(q_sol <= self.upper_limits + 1e-4))
        ee_pos, ee_quat = self.get_ee_pose()
        pos_err = float(np.linalg.norm(ee_pos - target_pos))
        ori_err = float(quat_distance(ee_quat, target_quat))

        if not in_limits:
            return False, pos_err, ori_err, "joint_limit_violation"
        if pos_err > 0.03:
            return False, pos_err, ori_err, f"fk_position_error_{pos_err:.4f}m"
        if ori_err > 0.25:
            return False, pos_err, ori_err, f"fk_orientation_error_{ori_err:.4f}rad"
        return True, pos_err, ori_err, "ok"

    def numerical_position_jacobian(self, q: np.ndarray, eps: float = 1e-4) -> np.ndarray:
        jac = np.zeros((3, len(q)), dtype=float)
        p0, _ = self.kin_solver.compute_forward_kinematics(self.ee_frame, q)
        p0 = np.asarray(p0, dtype=float)

        for i in range(len(q)):
            q_pert = q.copy()
            q_pert[i] += eps
            p1, _ = self.kin_solver.compute_forward_kinematics(self.ee_frame, q_pert)
            p1 = np.asarray(p1, dtype=float)
            jac[:, i] = (p1 - p0) / eps
        return jac

    def replay_joint_path(self, q_log: np.ndarray, t: np.ndarray, loops: int = 1) -> None:
        print("\nReplaying trajectory in Isaac Sim viewport...")
        for _ in range(loops):
            for i in range(len(q_log)):
                self.arm.set_joint_positions(q_log[i], joint_indices=self.arm_joint_indices)
                if i < len(q_log) - 1:
                    dwell = max(1, int(round((t[i + 1] - t[i]) / self.config.physics_dt)))
                else:
                    dwell = 2
                self.step(dwell)
