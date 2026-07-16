from pathlib import Path
from typing import Literal

from fastapi import APIRouter
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field

from api.dependencies import RobotCatalogServiceDep
from robots.catalog.assets import resolve_robot_asset_path, resolve_robot_urdf_path
from robots.catalog.types import RobotCatalogDefinition
from schemas.robot import RobotType


class RobotCatalogDefinitionResponse(BaseModel):
    type: RobotType = Field(..., description="Stable backend robot type identifier")
    display_name: str = Field(..., description="Human-readable robot type label")
    role: Literal["follower", "leader"] = Field(..., description="Default robot role")
    urdf_path: str = Field(description="URDF URL used by the UI model loader")
    package_map: dict[str, str] = Field(default_factory=dict, description="URDF package name to URL prefix map")
    joint_map: dict[str, list[str]] = Field(
        description="Observation joint name to URDF joint(s) mapping",
    )


def _to_response(definition: RobotCatalogDefinition) -> RobotCatalogDefinitionResponse:
    return RobotCatalogDefinitionResponse(
        type=definition.type,
        display_name=definition.display_name,
        role=definition.role,
        urdf_path=definition.urdf_path,
        package_map=definition.package_map,
        joint_map=definition.joint_map,
    )


router = APIRouter(prefix="/api/robots/catalog", tags=["Robot Catalog"])

# Robot assets are edited in place (a new gripper mesh, a re-exported URDF) while the UI
# keeps running. FileResponse sends only etag/last-modified, and with no Cache-Control a
# browser falls back to heuristic freshness -- 10% of the file's age, so a URDF checked out
# days ago is served from disk cache for hours without ever asking us, and edits appear to
# do nothing. "no-cache" forces the browser to ask on every load. FileResponse does not
# answer conditional requests, so each load is a full transfer; these are local-only
# megabyte-scale files, so correctness wins over the saved bytes.
_REVALIDATE = {"Cache-Control": "no-cache"}


@router.get("")
async def list_robot_catalog(catalog_service: RobotCatalogServiceDep) -> list[RobotCatalogDefinitionResponse]:
    """List robot catalog definitions exposed to the UI."""
    return [_to_response(definition) for definition in catalog_service.list_entries()]


@router.get("/{robot_type}/urdf")
async def get_robot_catalog_urdf(catalog_service: RobotCatalogServiceDep, robot_type: RobotType) -> FileResponse:
    """Return the URDF file for a catalog robot type."""
    definition = catalog_service.get_definition(robot_type)

    resolved_path = resolve_robot_urdf_path(definition)
    return FileResponse(resolved_path, headers=_REVALIDATE)


@router.get("/{robot_type}/{asset_path:path}")
async def get_robot_catalog_asset(
    catalog_service: RobotCatalogServiceDep,
    robot_type: RobotType,
    asset_path: Path,
) -> FileResponse:
    """Return an asset file referenced by a catalog robot URDF."""
    definition = catalog_service.get_definition(robot_type)

    resolved_path = resolve_robot_asset_path(definition, asset_path=asset_path)
    return FileResponse(resolved_path, headers=_REVALIDATE)
