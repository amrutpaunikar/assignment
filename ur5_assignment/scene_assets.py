import os
from dataclasses import dataclass
from typing import List, Sequence

import numpy as np
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage


@dataclass(frozen=True)
class PropSpec:
    name: str
    usd_path: str
    position: Sequence[float]
    orientation_wxyz: Sequence[float] = (1.0, 0.0, 0.0, 0.0)


class SceneAssetComposer:
    def __init__(self):
        self.props: List[PropSpec] = [
            PropSpec(
                name="cracker_box",
                usd_path=r"C:\isaacsim\assets\Isaac\5.1\Isaac\Props\YCB\Axis_Aligned_Physics\003_cracker_box.usd",
                position=(0.45, -0.20, 0.06),
            ),
            PropSpec(
                name="sugar_box",
                usd_path=r"C:\isaacsim\assets\Isaac\5.1\Isaac\Props\YCB\Axis_Aligned_Physics\004_sugar_box.usd",
                position=(0.50, -0.05, 0.06),
            ),
            PropSpec(
                name="tomato_soup_can",
                usd_path=r"C:\isaacsim\assets\Isaac\5.1\Isaac\Props\YCB\Axis_Aligned_Physics\005_tomato_soup_can.usd",
                position=(0.55, 0.10, 0.06),
            ),
            PropSpec(
                name="mustard_bottle",
                usd_path=r"C:\isaacsim\assets\Isaac\5.1\Isaac\Props\YCB\Axis_Aligned_Physics\006_mustard_bottle.usd",
                position=(0.62, -0.12, 0.06),
            ),
        ]

    def populate_props(self, parent_prim_path: str = "/World/Props") -> None:
        parent_prim = SingleXFormPrim(parent_prim_path, name="props_root")
        parent_prim.set_world_pose(position=np.array([0.0, 0.0, 0.0]), orientation=np.array([1.0, 0.0, 0.0, 0.0]))

        for spec in self.props:
            if not os.path.exists(spec.usd_path):
                print(f"Skipping missing prop: {spec.usd_path}")
                continue

            prim_path = f"{parent_prim_path}/{spec.name}"
            add_reference_to_stage(usd_path=spec.usd_path, prim_path=prim_path)
            prop_prim = SingleXFormPrim(prim_path, name=f"{spec.name}_xform")
            prop_prim.set_world_pose(
                position=np.asarray(spec.position, dtype=float),
                orientation=np.asarray(spec.orientation_wxyz, dtype=float),
            )
