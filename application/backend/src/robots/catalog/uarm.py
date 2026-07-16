from schemas.robot import RobotType

from .types import RobotCatalogDefinition

# The uArm leader publishes normalized j{n}.pos features; the stub URDF joints
# are named j1..j6 so the mapping is 1:1.
_UARM_TO_URDF = {
    "j1.pos": ["j1"],
    "j2.pos": ["j2"],
    "j3.pos": ["j3"],
    "j4.pos": ["j4"],
    "j5.pos": ["j5"],
    "j6.pos": ["j6"],
}


def get_definitions() -> list[RobotCatalogDefinition]:
    """Return the built-in uArm leader catalog definition.

    Uses a self-contained primitive-geometry stub URDF under
    ``static/robot-assets/uarm/`` (no external meshes). Replace it with a real
    Config2 URDF later while keeping the j1..j6 joint names and this entry.
    """
    return [
        RobotCatalogDefinition(
            type=RobotType.UARM_LEADER,
            display_name="uArm Leader",
            role="leader",
            urdf_path=f"/api/robots/catalog/{RobotType.UARM_LEADER}/urdf",
            package_map={},
            joint_map=_UARM_TO_URDF,
            urdf_relative_path="uarm/urdf/uarm_leader.urdf",
        ),
    ]
