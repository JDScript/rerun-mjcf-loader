import rerun as rr
from pathlib import Path
from natsort import natsorted
import pickle

import sys

sys.path.append("src")
from rerun_mjcf_logger.logger import RerunMJCFLogger, build_scene
from rerun_mjcf_logger.utils import log_angle_rot


class RecordingVisualizer:
    def blueprint_raw(self):
        from rerun.blueprint import (
            Blueprint,
            BlueprintPanel,
            Horizontal,
            Vertical,
            SelectionPanel,
            Spatial2DView,
            Spatial3DView,
            TimePanel,
            TimeSeriesView,
            Tabs,
        )

        blueprint = Blueprint(
            Horizontal(
                Vertical(
                    Spatial3DView(name="robot view", origin="/", contents=["/**"]),
                    Horizontal(
                        Spatial2DView(
                            name="camera/base/rgb",
                            origin="camera/base_rgb",
                        ),
                        Spatial2DView(
                            name="camera/base/depth",
                            origin="camera/base_depth",
                        ),
                    ),
                    row_shares=[1, 1],
                ),
                Tabs(
                    Vertical(
                        *(
                            TimeSeriesView(origin=f"action/joint_position/{i}")
                            for i in range(6)
                        ),
                        name="action/joint_position",
                    ),
                ),
                column_shares=[3, 2],
            ),
            BlueprintPanel(expanded=False),
            SelectionPanel(expanded=False),
            TimePanel(expanded=False),
        )
        return blueprint

    def playback(
        self,
        recording_path: Path,
        mjcf_logger: RerunMJCFLogger,
    ):
        rr.init(application_id=recording_path.name, spawn=False)
        rr.connect_grpc("rerun+http://localhost:9876/proxy")
        rr.send_blueprint(self.blueprint_raw())

        rr.set_time("recording", sequence=0)
        joints = mjcf_logger.log()

        all_logs = natsorted(recording_path.glob("*.pkl"))
        for step_idx, log_path in enumerate(all_logs):
            with open(log_path, "rb") as f:
                log = pickle.load(f)

            rr.set_time("recording", sequence=step_idx + 1)
            joint_positions = log["joint_positions"]

            for joint_idx, joint_angle in enumerate(joint_positions[:6]):
                log_angle_rot(
                    entity_to_transform=joints,
                    link=joint_idx,
                    angle_rad=joint_angle,
                )

            rr.log("camera/base_rgb", rr.Image(log["base_rgb"]))
            rr.log("camera/base_depth", rr.DepthImage(log["base_depth"], meter=1000, depth_range=[0, 4000]))

            for action_idx, action in enumerate(log["control"]):
                rr.log(f"action/joint_position/{action_idx}", rr.Scalars(action))


if __name__ == "__main__":
    ROBOT_XML_PATH = "./third_party/mujoco_menagerie/universal_robots_ur10e/ur10e.xml"
    GRIPPER_XML_PATH = "./third_party/mujoco_menagerie/robotiq_2f85/2f85.xml"

    arena, assets = build_scene(ROBOT_XML_PATH, GRIPPER_XML_PATH)
    mjcf_logger = RerunMJCFLogger(
        xml_string=arena.to_xml_string(),
        assets=assets,
    )

    vis = RecordingVisualizer()
    vis.playback(
        recording_path=Path("./bc_data/gello/0430_213005"),
        mjcf_logger=mjcf_logger,
    )
