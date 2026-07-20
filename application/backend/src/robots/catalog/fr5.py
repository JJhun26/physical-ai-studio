from schemas.robot import RobotType

from .types import RobotCatalogDefinition

# FR5 URDF joints are already named j1..j6, matching the normalized j{n}.pos
# features published by the FR5 follower / uArm leader clients. gripper.pos drives
# both PGEA carriages together (0 = closed, +0.020 m = fully open), the same
# single-feature-to-two-joints mapping the WidowX follower uses.
_FR5_TO_URDF = {
    "j1.pos": ["j1"],
    "j2.pos": ["j2"],
    "j3.pos": ["j3"],
    "j4.pos": ["j4"],
    "j5.pos": ["j5"],
    "j6.pos": ["j6"],
    "gripper.pos": ["left_carriage_joint", "right_carriage_joint"],
}


def get_definitions() -> list[RobotCatalogDefinition]:
    """Return the built-in Fairino FR5 + PGEA-100-40 follower catalog definition.

    The follower carries a DH Robotics PGEA-100-40-W-F parallel gripper on its
    wrist3 flange, so the catalog serves a combined ``fr5_pgea`` model rather than
    the bare arm. Assets live under ``static/robot-assets/fr5_pgea/``:

    - ``urdf/fr5_pgea.urdf`` and ``meshes/pgea/`` (gripper body + two jaws, derived
      from the vendor STEP) are hand-authored and version controlled.
    - ``meshes/fairino5_v6/`` are the FR5 arm STLs from FAIR-INNOVATION/frcobot_ros2
      (``fairino_description``), synced by ``sync_robot_assets`` rather than committed.

    The URDF references ``package://fr5_pgea/...`` which the UI rewrites via
    ``package_map`` onto the catalog asset endpoint.
    """
    return [
        RobotCatalogDefinition(
            type=RobotType.FR5_FOLLOWER,
            display_name="Fairino FR5 Follower",
            role="follower",
            urdf_path=f"/api/robots/catalog/{RobotType.FR5_FOLLOWER}/urdf",
            package_map={"fr5_pgea": f"/api/robots/catalog/{RobotType.FR5_FOLLOWER}"},
            joint_map=_FR5_TO_URDF,
            urdf_relative_path="fr5_pgea/urdf/fr5_pgea.urdf",
        ),
    ]
