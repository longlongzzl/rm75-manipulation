"""Magnetic-panel structure frontend, rules, geometry, and execution facade."""

from .catalog import (
    catalog_from_scene_metadata,
    inventory_from_scene,
    load_magnetic_catalog,
    save_magnetic_catalog,
)
from .contracts import (
    MagneticAssemblySpec,
    MagneticConnection,
    MagneticInventoryItem,
    MagneticJointType,
    MagneticPanelSpec,
    MagneticPiece,
    PanelEdge,
    PanelPoseClass,
    PanelRole,
    ResolvedMagneticPlacement,
)
from .geometry import (
    MagneticGeometryReport,
    MagneticGeometryValidator,
    MagneticGeometryViolation,
    StrictMagneticAssemblyPlanner,
)
from .llm import (
    MagneticAssemblyFrontend,
    OpenAICompatibleStructureClient,
    StructureLLM,
)
from .planner import (
    MagneticAssemblyPlanner,
    MagneticAssemblySystem,
    PreparedMagneticProgram,
)
from .rules import (
    MagneticAssemblyRules,
    MagneticValidationReport,
    RuleViolation,
)

__all__ = [
    "MagneticAssemblyFrontend",
    "MagneticAssemblyPlanner",
    "MagneticAssemblyRules",
    "MagneticAssemblySpec",
    "MagneticAssemblySystem",
    "MagneticConnection",
    "MagneticGeometryReport",
    "MagneticGeometryValidator",
    "MagneticGeometryViolation",
    "MagneticInventoryItem",
    "MagneticJointType",
    "MagneticPanelSpec",
    "MagneticPiece",
    "MagneticValidationReport",
    "OpenAICompatibleStructureClient",
    "PanelEdge",
    "PanelPoseClass",
    "PanelRole",
    "PreparedMagneticProgram",
    "ResolvedMagneticPlacement",
    "RuleViolation",
    "StrictMagneticAssemblyPlanner",
    "StructureLLM",
    "catalog_from_scene_metadata",
    "inventory_from_scene",
    "load_magnetic_catalog",
    "save_magnetic_catalog",
]
