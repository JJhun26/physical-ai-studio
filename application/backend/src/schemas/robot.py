from datetime import datetime
from enum import StrEnum
from typing import Annotated, Literal
from uuid import UUID

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, model_validator

from schemas.base import BaseIDModel


class SerialPortInfo(BaseModel):
    connection_string: str | None
    serial_number: str | None


class RobotType(StrEnum):
    SO101_FOLLOWER = "SO101_Follower"
    SO101_LEADER = "SO101_Leader"
    TROSSEN_WIDOWXAI_LEADER = "Trossen_WidowXAI_Leader"
    TROSSEN_WIDOWXAI_FOLLOWER = "Trossen_WidowXAI_Follower"
    TROSSEN_BIMANUAL_WIDOWXAI_LEADER = "Trossen_Bimanual_WidowXAI_Leader"
    TROSSEN_BIMANUAL_WIDOWXAI_FOLLOWER = "Trossen_Bimanual_WidowXAI_Follower"
    # Fairino FR5 (Ethernet / Fairino SDK) follower, uArm (Feetech serial) leader.
    # Both expose 6 normalized joints (j1.pos..j6.pos) so the name-matched
    # TeleoperateWorker can drive FR5 from the uArm exactly like SO-ARM.
    FR5_FOLLOWER = "FR5_Follower"
    UARM_LEADER = "UArm_Leader"


# ============================================================================
# Payload Models (Configuration Only)
# ============================================================================


class SO101RobotPayload(BaseModel):
    """Connection configuration for SO-101 serial robots."""

    connection_string: str = Field(
        default="",
        description="Serial port path; leave empty to auto-discover via serial_number",
    )
    serial_number: str = Field(default="", description="USB serial number of the robot (when available)")

    @model_validator(mode="after")
    def validate_identifier(self) -> "SO101RobotPayload":
        if self.connection_string == "" and self.serial_number == "":
            raise ValueError("Either serial_number or connection_string is required for SO101 robots")
        return self


class TrossenSingleArmPayload(BaseModel):
    """Connection configuration for Trossen single-arm robots."""

    connection_string: str = Field(..., description="IP address of the robot")
    serial_number: str = Field(default="", description="Serial number (unused for IP robots)")


class TrossenBimanualPayload(BaseModel):
    """Connection configuration for Trossen bimanual robots."""

    connection_string_left: str = Field(..., description="IP address of the left arm")
    connection_string_right: str = Field(..., description="IP address of the right arm")
    serial_number: str = Field(default="", description="Serial number (unused for IP robots)")


class FR5FollowerPayload(BaseModel):
    """Connection configuration for a Fairino FR5 follower (Ethernet / Fairino SDK)."""

    connection_string: str = Field(
        default="192.168.58.2",
        description="Controller IP address (Fairino SDK, port 20003)",
    )
    serial_number: str = Field(default="", description="Serial number (unused for IP robots)")


class UArmLeaderPayload(BaseModel):
    """Connection configuration for a uArm leader (Feetech STS3215 serial bus)."""

    connection_string: str = Field(
        default="",
        description="Serial port path (e.g. /dev/ttyACM0); leave empty to auto-discover via serial_number",
    )
    serial_number: str = Field(default="", description="USB serial number of the leader (when available)")

    @model_validator(mode="after")
    def validate_identifier(self) -> "UArmLeaderPayload":
        if self.connection_string == "" and self.serial_number == "":
            raise ValueError("Either serial_number or connection_string is required for uArm leader")
        return self


# ============================================================================
# Concrete Robot Models
# ============================================================================


_SO101Types = Literal[RobotType.SO101_FOLLOWER, RobotType.SO101_LEADER]
_TrossenTypes = Literal[RobotType.TROSSEN_WIDOWXAI_LEADER, RobotType.TROSSEN_WIDOWXAI_FOLLOWER]
_TrossenBimanualTypes = Literal[
    RobotType.TROSSEN_BIMANUAL_WIDOWXAI_LEADER, RobotType.TROSSEN_BIMANUAL_WIDOWXAI_FOLLOWER
]
_FR5Types = Literal[RobotType.FR5_FOLLOWER]
_UArmTypes = Literal[RobotType.UARM_LEADER]


class BaseRobot(BaseIDModel):
    id: Annotated[UUID, Field(description="Unique identifier")]
    created_at: datetime | None = Field(None)
    updated_at: datetime | None = Field(None)

    name: str = Field(..., description="Human-readable robot name")
    active_calibration_id: UUID | None = Field(default=None, description="The ID of the active calibration")


class SO101Robot(BaseRobot):
    """SO-101 follower or leader robot using a serial connection."""

    type: _SO101Types = Field(..., description="Type of robot configuration")
    payload: SO101RobotPayload = Field(..., description="SO-101 connection configuration")

    model_config = ConfigDict(
        json_schema_extra={
            "example": {
                "id": "a5e2cde6-936b-4a9e-a213-08dda0afa453",
                "name": "Assembly Line Robot 1",
                "type": "SO101_Follower",
                "payload": {
                    "connection_string": "",
                    "serial_number": "SO101-2024-001",
                },
                "active_calibration_id": "b7f3d9e2-1a2b-4c3d-8e9f-0a1b2c3d4e5f",
                "created_at": "2024-01-15T10:30:00Z",
                "updated_at": "2024-01-15T10:30:00Z",
            },
        },
    )


class TrossenSingleArmRobot(BaseRobot):
    """Trossen WidowX AI follower or leader robot using an IP connection."""

    type: _TrossenTypes = Field(..., description="Type of robot configuration")
    payload: TrossenSingleArmPayload = Field(..., description="Trossen single-arm connection configuration")

    model_config = ConfigDict(
        json_schema_extra={
            "example": {
                "id": "a5e2cde6-936b-4a9e-a213-08dda0afa453",
                "name": "WidowX AI Robot 1",
                "type": "Trossen_WidowXAI_Follower",
                "payload": {
                    "connection_string": "192.168.1.100",
                    "serial_number": "",
                },
                "active_calibration_id": None,
                "created_at": "2024-01-15T10:30:00Z",
                "updated_at": "2024-01-15T10:30:00Z",
            },
        },
    )


class TrossenBimanualRobot(BaseRobot):
    """Trossen Bimanual WidowX AI robot using two IP connections (left + right)."""

    type: _TrossenBimanualTypes = Field(..., description="Type of robot configuration")
    payload: TrossenBimanualPayload = Field(..., description="Trossen bimanual connection configuration")

    model_config = ConfigDict(
        json_schema_extra={
            "example": {
                "id": "a5e2cde6-936b-4a9e-a213-08dda0afa454",
                "name": "WidowX AI Bimanual Robot 1",
                "type": "Trossen_Bimanual_WidowXAI_Follower",
                "payload": {
                    "connection_string_left": "192.168.1.100",
                    "connection_string_right": "192.168.1.101",
                    "serial_number": "",
                },
                "active_calibration_id": None,
                "created_at": "2024-01-15T10:30:00Z",
                "updated_at": "2024-01-15T10:30:00Z",
            },
        },
    )


class FR5Robot(BaseRobot):
    """Fairino FR5 follower robot using an Ethernet (Fairino SDK) connection."""

    type: _FR5Types = Field(..., description="Type of robot configuration")
    payload: FR5FollowerPayload = Field(..., description="FR5 connection configuration")

    model_config = ConfigDict(
        json_schema_extra={
            "example": {
                "id": "a5e2cde6-936b-4a9e-a213-08dda0afa455",
                "name": "FR5 Follower",
                "type": "FR5_Follower",
                "payload": {"connection_string": "192.168.58.2", "serial_number": ""},
                "active_calibration_id": None,
                "created_at": "2024-01-15T10:30:00Z",
                "updated_at": "2024-01-15T10:30:00Z",
            },
        },
    )


class UArmRobot(BaseRobot):
    """uArm leader robot (Feetech STS3215 bus) using a serial connection."""

    type: _UArmTypes = Field(..., description="Type of robot configuration")
    payload: UArmLeaderPayload = Field(..., description="uArm connection configuration")

    model_config = ConfigDict(
        json_schema_extra={
            "example": {
                "id": "a5e2cde6-936b-4a9e-a213-08dda0afa456",
                "name": "uArm Leader",
                "type": "UArm_Leader",
                "payload": {"connection_string": "/dev/ttyACM0", "serial_number": "5B3D045122"},
                "active_calibration_id": "b7f3d9e2-1a2b-4c3d-8e9f-0a1b2c3d4e5f",
                "created_at": "2024-01-15T10:30:00Z",
                "updated_at": "2024-01-15T10:30:00Z",
            },
        },
    )


# Discriminated union of all robot types
Robot = Annotated[
    SO101Robot | TrossenSingleArmRobot | TrossenBimanualRobot | FR5Robot | UArmRobot,
    Field(discriminator="type"),
]

RobotAdapter: TypeAdapter[Robot] = TypeAdapter(Robot)


# ============================================================================
# RobotWithConnectionState variants
# ============================================================================

_ConnectionStatus = Literal["online", "offline", "unknown"]


class SO101RobotWithConnectionState(SO101Robot):
    connection_status: _ConnectionStatus = "unknown"


class TrossenSingleArmRobotWithConnectionState(TrossenSingleArmRobot):
    connection_status: _ConnectionStatus = "unknown"


class TrossenBimanualRobotWithConnectionState(TrossenBimanualRobot):
    connection_status: _ConnectionStatus = "unknown"


class FR5RobotWithConnectionState(FR5Robot):
    connection_status: _ConnectionStatus = "unknown"


class UArmRobotWithConnectionState(UArmRobot):
    connection_status: _ConnectionStatus = "unknown"


RobotWithConnectionState = Annotated[
    SO101RobotWithConnectionState
    | TrossenSingleArmRobotWithConnectionState
    | TrossenBimanualRobotWithConnectionState
    | FR5RobotWithConnectionState
    | UArmRobotWithConnectionState,
    Field(discriminator="type"),
]

RobotWithConnectionStateAdapter: TypeAdapter[RobotWithConnectionState] = TypeAdapter(RobotWithConnectionState)
