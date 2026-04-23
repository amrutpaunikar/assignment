from dataclasses import dataclass

import numpy as np


@dataclass
class Objective2Result:
    positions: np.ndarray
    quats: np.ndarray
    q_log: np.ndarray
    sigma_log: np.ndarray
