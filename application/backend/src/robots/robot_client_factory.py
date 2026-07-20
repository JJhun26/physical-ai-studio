import os
from typing import Literal

from loguru import logger
from physicalai.robot.so101 import SO101, SO101Calibration
from physicalai.robot.trossen import BimanualWidowXAI, WidowXAI

from exceptions import ResourceNotFoundError, ResourceType
from robots.drivers.fr5_follower import FR5FollowerClient, FR5FollowerConfig
from robots.drivers.uarm_leader import JointCalibration, UArmLeaderClient, UArmLeaderConfig
from robots.physicalai_adapter import PhysicalAIRobotAdapter, PhysicalAIRobotAdapterConfig
from robots.robot_client import RobotClient
from schemas.calibration import Calibration
from schemas.robot import FR5Robot, Robot, RobotType, SO101Robot, TrossenBimanualRobot, UArmRobot
from services.robot_calibration_service import RobotCalibrationService
from utils.serial_robot_tools import RobotConnectionManager, find_so101_port, serial_port_from_so101


def _env_int(name: str) -> int | None:
    raw = os.environ.get(name)
    if raw is None or raw.strip() == "":
        return None
    try:
        return int(raw)
    except ValueError:
        return None


class RobotClientFactory:
    calibration_service: RobotCalibrationService
    robot_manager: RobotConnectionManager

    def __init__(
        self,
        robot_manager: RobotConnectionManager,
        calibration_service: RobotCalibrationService,
    ) -> None:
        self.robot_manager = robot_manager
        self.calibration_service = calibration_service

    async def build(self, robot: Robot) -> RobotClient:
        match robot.type:
            case RobotType.TROSSEN_WIDOWXAI_FOLLOWER:
                robot_driver = WidowXAI(ip=robot.payload.connection_string, role="follower")
                return PhysicalAIRobotAdapter(
                    robot=robot_driver,
                    robot_type=RobotType.TROSSEN_WIDOWXAI_FOLLOWER,
                    config=PhysicalAIRobotAdapterConfig(
                        include_velocities=True,
                        goal_time_scale=1.0,
                        external_effort_gain=0.1,
                    ),
                )
            case RobotType.TROSSEN_WIDOWXAI_LEADER:
                robot_driver = WidowXAI(ip=robot.payload.connection_string, role="leader")
                return PhysicalAIRobotAdapter(
                    robot=robot_driver,
                    robot_type=RobotType.TROSSEN_WIDOWXAI_LEADER,
                    config=PhysicalAIRobotAdapterConfig(
                        include_velocities=True,
                        goal_time_scale=1.0,
                        external_effort_gain=0.1,
                    ),
                )
            case RobotType.TROSSEN_BIMANUAL_WIDOWXAI_FOLLOWER:
                return self._build_bimanual_widowxai(robot, mode="follower")
            case RobotType.TROSSEN_BIMANUAL_WIDOWXAI_LEADER:
                return self._build_bimanual_widowxai(robot, mode="leader")
            case RobotType.SO101_FOLLOWER:
                return await self._build_so101(robot)
            case RobotType.SO101_LEADER:
                return await self._build_so101(robot)
            case RobotType.FR5_FOLLOWER:
                return self._build_fr5(robot)
            case RobotType.UARM_LEADER:
                return await self._build_uarm(robot)
            case _:
                raise ValueError(f"Unsupported robot type: {robot.type}")

    @staticmethod
    def _build_fr5(robot: FR5Robot) -> FR5FollowerClient:
        ip = robot.payload.connection_string or "192.168.58.2"
        # The gripper joins the pipeline only when something can actually drive it: the
        # uArm's trigger, configured by the same env pair the leader reads. Without it
        # nothing ever commands the gripper, while gripper.pos would still enter the
        # feature list -- so recording would write a column holding the follower's
        # *assumed* opening (a constant) as the observation and nothing at all as the
        # action, which is how dataset writes failed with KeyError: 'gripper.pos'.
        # A constant guess is worse in a dataset than an absent column.
        gripper = _env_int("UARM_TRIGGER_RAW_OPEN") is not None and _env_int("UARM_TRIGGER_RAW_CLOSE") is not None
        if not gripper:
            logger.info("uArm trigger not configured; FR5 gripper left out of the feature set")
        return FR5FollowerClient(FR5FollowerConfig(ip=ip, gripper_enabled=gripper))

    async def _build_uarm(self, robot: UArmRobot) -> UArmLeaderClient:
        port = robot.payload.connection_string
        if not port:
            raise ResourceNotFoundError(ResourceType.ROBOT, robot.payload.serial_number)

        joints: list[JointCalibration] | None = None
        calibration = await self._get_uarm_calibration(robot)
        if calibration is not None:
            # Calibration.values is keyed by joint name (j1..j6); order by name.
            joints = []
            for name in (f"j{i}" for i in range(1, 7)):
                val = calibration.values.get(name)
                if val is None:
                    raise ValueError(f"uArm calibration missing joint '{name}'")
                joints.append(
                    JointCalibration(
                        motor_id=val.id,
                        range_min=val.range_min,
                        range_max=val.range_max,
                        drive_mode=val.drive_mode,
                        homing_offset=val.homing_offset,
                    )
                )

        # Trigger (servo id 7) -> follower gripper. Its raw endpoints are a per-rig
        # calibration, supplied via env like the SDK override; absent -> the trigger
        # is not read and the follower just holds its gripper.
        raw_open = _env_int("UARM_TRIGGER_RAW_OPEN")
        raw_close = _env_int("UARM_TRIGGER_RAW_CLOSE")

        return UArmLeaderClient(
            UArmLeaderConfig(port=port, joints=joints, gripper_raw_open=raw_open, gripper_raw_close=raw_close)
        )

    async def _get_uarm_calibration(self, robot: UArmRobot) -> Calibration | None:
        if robot.active_calibration_id is None:
            return None
        return await self.calibration_service.get_calibration(robot.active_calibration_id)

    @staticmethod
    def _build_bimanual_widowxai(
        robot: TrossenBimanualRobot, mode: Literal["follower", "leader"]
    ) -> PhysicalAIRobotAdapter:
        left_driver = WidowXAI(ip=robot.payload.connection_string_left, role=mode)
        right_driver = WidowXAI(ip=robot.payload.connection_string_right, role=mode)
        bimanual_robot = BimanualWidowXAI(left=left_driver, right=right_driver)
        robot_type = (
            RobotType.TROSSEN_BIMANUAL_WIDOWXAI_FOLLOWER
            if mode == "follower"
            else RobotType.TROSSEN_BIMANUAL_WIDOWXAI_LEADER
        )
        return PhysicalAIRobotAdapter(
            robot=bimanual_robot,
            robot_type=robot_type,
            config=PhysicalAIRobotAdapterConfig(
                include_velocities=True,
                goal_time_scale=1.0,
                external_effort_gain=0.1,
            ),
        )

    async def _build_so101(self, robot: SO101Robot) -> PhysicalAIRobotAdapter:
        port = await self.find_robot_port(robot)
        calibration = await self._get_robot_calibration(robot)

        if calibration is None:
            raise ResourceNotFoundError(ResourceType.ROBOT_CALIBRATION, robot.payload.serial_number)
        if port is None:
            raise ResourceNotFoundError(ResourceType.ROBOT, robot.payload.serial_number)

        role = "follower" if robot.type == RobotType.SO101_FOLLOWER else "leader"

        so101_cal = SO101Calibration.from_dict(
            {
                name: {
                    "id": val.id,
                    "drive_mode": val.drive_mode,
                    "homing_offset": val.homing_offset,
                    "range_min": val.range_min,
                    "range_max": val.range_max,
                }
                for name, val in calibration.values.items()
            }
        )

        so101 = SO101(port=port, calibration=so101_cal, role=role, unit="normalized")
        return PhysicalAIRobotAdapter(
            robot=so101,
            robot_type=robot.type,
            config=PhysicalAIRobotAdapterConfig(
                include_velocities=False,
                goal_time_scale=1.0,
                external_effort_gain=None,
            ),
        )

    async def find_robot_port(self, robot: SO101Robot) -> str:
        port = await find_so101_port(self.robot_manager, serial_port_from_so101(robot))
        if port is None:
            resource_key = robot.payload.serial_number or robot.payload.connection_string
            raise ResourceNotFoundError(ResourceType.ROBOT, resource_key)
        return port

    async def _get_robot_calibration(self, robot: SO101Robot) -> Calibration | None:
        if robot.active_calibration_id is None:
            return None

        return await self.calibration_service.get_calibration(robot.active_calibration_id)
