import asyncio
import sys

from schemas import Robot
from schemas.robot import FR5Robot, SO101Robot, TrossenBimanualRobot, TrossenSingleArmRobot


class IPDiscovery:
    @staticmethod
    def _ping_command(ip: str, ping_timeout: float) -> list[str]:
        """Build a single-shot ping bounded by ping_timeout.

        The timeout flag differs per platform and is not interchangeable:
        Windows takes ``-w`` in milliseconds, BSD/macOS ``-W`` in milliseconds, and
        iputils (Linux) ``-W`` in *seconds*. Passing milliseconds to Linux turns a
        1s budget into a ~17 minute hang.
        """
        if sys.platform.lower().startswith("win"):
            return ["ping", "-n", "1", "-w", str(int(ping_timeout * 1000)), ip]
        if sys.platform == "darwin":
            return ["ping", "-c", "1", "-W", str(int(ping_timeout * 1000)), ip]
        return ["ping", "-c", "1", "-W", str(max(1, round(ping_timeout))), ip]

    @staticmethod
    async def ping(ip: str, ping_timeout: float = 1.0) -> bool:
        """Async ping using system ping command.
        Works on macOS/Linux/Windows.
        """
        proc = await asyncio.create_subprocess_exec(
            *IPDiscovery._ping_command(ip, ping_timeout),
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL,
        )
        # Belt and braces: an unreachable host must never hold the online endpoint
        # open, whatever the local ping does with its own timeout flag.
        try:
            return (await asyncio.wait_for(proc.wait(), timeout=ping_timeout + 1.0)) == 0
        except TimeoutError:
            proc.kill()
            await proc.wait()
            return False

    async def is_reachable(self, robot: Robot) -> bool:
        if not isinstance(robot, SO101Robot | TrossenSingleArmRobot | FR5Robot):
            return False
        if not robot.payload.connection_string:
            return False
        return await self.ping(robot.payload.connection_string)

    async def is_reachable_bimanual(self, robot: Robot) -> bool:
        """Ping both arms of a bimanual robot; returns True only if both are reachable."""
        if not isinstance(robot, TrossenBimanualRobot):
            return False
        left = robot.payload.connection_string_left
        right = robot.payload.connection_string_right
        if not left or not right:
            return False
        left_ok, right_ok = await asyncio.gather(self.ping(left), self.ping(right))
        return left_ok and right_ok
