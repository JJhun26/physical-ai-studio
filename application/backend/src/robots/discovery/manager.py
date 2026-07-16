from robots.discovery.ip import IPDiscovery
from robots.discovery.serial import SerialDiscovery
from schemas import Robot, SerialPortInfo
from schemas.robot import RobotType, SO101Robot, UArmRobot
from utils.serial_robot_tools import RobotConnectionManager, find_so101_port, serial_port_from_so101

_IP_SINGLE_TYPES = {
    RobotType.TROSSEN_WIDOWXAI_LEADER,
    RobotType.TROSSEN_WIDOWXAI_FOLLOWER,
    RobotType.FR5_FOLLOWER,
}
_IP_BIMANUAL_TYPES = {RobotType.TROSSEN_BIMANUAL_WIDOWXAI_LEADER, RobotType.TROSSEN_BIMANUAL_WIDOWXAI_FOLLOWER}


class DiscoveryManager:
    def __init__(self):
        self.serial = SerialDiscovery()
        self.ip = IPDiscovery()
        self.serial_manager = RobotConnectionManager()

    async def refresh_hardware_ports(self) -> None:
        await self.serial_manager.find_robots()

    async def is_robot_online(self, robot: Robot) -> bool:
        if robot.type in {RobotType.SO101_LEADER, RobotType.SO101_FOLLOWER}:
            if not isinstance(robot, SO101Robot):
                return False
            return await find_so101_port(self.serial_manager, serial_port_from_so101(robot)) is not None
        if robot.type == RobotType.UARM_LEADER:
            if not isinstance(robot, UArmRobot):
                return False
            info = SerialPortInfo(
                connection_string=robot.payload.connection_string or None,
                serial_number=robot.payload.serial_number or None,
            )
            return await find_so101_port(self.serial_manager, info) is not None
        if robot.type in _IP_SINGLE_TYPES:
            return await self.ip.is_reachable(robot)
        if robot.type in _IP_BIMANUAL_TYPES:
            return await self.ip.is_reachable_bimanual(robot)
        return False
