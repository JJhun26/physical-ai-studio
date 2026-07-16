from schemas.robot import RobotType

from .types import RobotCatalogDefinition

# FR5 URDF joints are already named j1..j6, matching the normalized j{n}.pos
# features published by the FR5 follower / uArm leader clients.
_FR5_TO_URDF = {
    "j1.pos": ["j1"],
    "j2.pos": ["j2"],
    "j3.pos": ["j3"],
    "j4.pos": ["j4"],
    "j5.pos": ["j5"],
    "j6.pos": ["j6"],
}


def get_definitions() -> list[RobotCatalogDefinition]:
    """Return built-in Fairino FR5 robot catalog definitions.

    Assets (fairino5_v6.urdf + STL meshes) live under
    ``static/robot-assets/fr5/`` and originate from FAIR-INNOVATION/frcobot_ros2
    (``fairino_description``). The URDF references
    ``package://fairino_description/...`` which the UI rewrites via ``package_map``.
    """
    return [
        RobotCatalogDefinition(
            type=RobotType.FR5_FOLLOWER,
            display_name="Fairino FR5 Follower",
            role="follower",
            urdf_path=f"/api/robots/catalog/{RobotType.FR5_FOLLOWER}/urdf",
            package_map={"fairino_description": f"/api/robots/catalog/{RobotType.FR5_FOLLOWER}"},
            joint_map=_FR5_TO_URDF,
            urdf_relative_path="fr5/urdf/fairino5_v6.urdf",
        ),
    ]
