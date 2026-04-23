from pathlib import Path

from .config import AssignmentConfig
from .io_utils import ensure_dir
from .objectives import Objective1WorkspaceReachability, Objective2ContinuousPath, Objective3TimeParameterization
from .robot_session import UR5RobotSession


class UR5AssignmentPipeline:
    def __init__(self, config: AssignmentConfig):
        self.config = config
        self.out_dir: Path = ensure_dir(config.out_dir)
        self.session = UR5RobotSession(config)

    def run(self) -> None:
        self.session.setup_scene()

        objective1 = Objective1WorkspaceReachability(self.session, self.config)
        objective2 = Objective2ContinuousPath(self.session, self.config)
        objective3 = Objective3TimeParameterization(self.session)

        objective1.run(self.out_dir)
        result2 = objective2.run(self.out_dir)
        t = objective3.run(self.out_dir, result2)

        self.session.replay_joint_path(result2.q_log, t, loops=self.config.playback_loops)

        print("\nDone.")
        print(f"All outputs saved in: {self.out_dir.resolve()}")
        print("Use Isaac Sim Recorder window for video capture in GUI mode.")
