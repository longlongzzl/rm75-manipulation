"""Natural-language frontend for bounded magnetic-panel structures.

The language model never emits robot trajectories. It emits a small symbolic
assembly graph that is parsed and checked by :class:`MagneticAssemblyRules`.
Invalid LLM output is repaired once and otherwise falls back to deterministic
templates.
"""

from __future__ import annotations

import json
import os
import re
import urllib.request
from dataclasses import dataclass
from typing import Any, Mapping, Protocol, Sequence

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
)
from .rules import MagneticAssemblyRules


class StructureLLM(Protocol):
    def complete(self, prompt: str) -> str: ...


@dataclass(frozen=True)
class OpenAICompatibleStructureClient:
    endpoint: str
    model: str
    api_key: str | None = None
    timeout_s: float = 60.0
    temperature: float = 0.1

    def complete(self, prompt: str) -> str:
        key = self.api_key or os.environ.get("OPENAI_API_KEY")
        headers = {"Content-Type": "application/json"}
        if key:
            headers["Authorization"] = f"Bearer {key}"
        payload = json.dumps(
            {
                "model": self.model,
                "temperature": self.temperature,
                "messages": [
                    {
                        "role": "system",
                        "content": (
                            "You compile descriptions into strict JSON magnetic "
                            "assembly graphs. Return JSON only."
                        ),
                    },
                    {"role": "user", "content": prompt},
                ],
            }
        ).encode("utf-8")
        request = urllib.request.Request(
            self.endpoint,
            data=payload,
            headers=headers,
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=self.timeout_s) as response:
            body = json.loads(response.read().decode("utf-8"))
        return str(body["choices"][0]["message"]["content"])


def _extract_json(text: str) -> Mapping[str, Any]:
    value = str(text).strip()
    fenced = re.search(r"```(?:json)?\s*(.*?)```", value, flags=re.S | re.I)
    if fenced:
        value = fenced.group(1).strip()
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError:
        start = value.find("{")
        end = value.rfind("}")
        if start < 0 or end <= start:
            raise
        parsed = json.loads(value[start : end + 1])
    if not isinstance(parsed, Mapping):
        raise ValueError("LLM assembly response must be a JSON object")
    return parsed


class MagneticAssemblyFrontend:
    def __init__(
        self,
        catalog: Mapping[str, MagneticPanelSpec],
        inventory: Sequence[MagneticInventoryItem],
        *,
        max_pieces: int = 12,
        rules: MagneticAssemblyRules | None = None,
    ) -> None:
        self.catalog = {str(key): value for key, value in catalog.items()}
        self.inventory = tuple(inventory)
        self.max_pieces = int(max_pieces)
        if self.max_pieces < 1 or self.max_pieces > 12:
            raise ValueError("magnetic frontend max_pieces must be in [1, 12]")
        self.rules = rules or MagneticAssemblyRules()

    def _prompt(self, description: str) -> str:
        inventory = [
            {
                "object_id": item.object_id,
                "asset_name": item.asset_name,
                "tray_slot": item.tray_slot,
                "size_xyz_m": list(
                    self.catalog[item.asset_name].size_xyz_m
                )
                if item.asset_name in self.catalog
                else None,
                "allowed_roles": [
                    role.value
                    for role in self.catalog[item.asset_name].allowed_roles
                ]
                if item.asset_name in self.catalog
                else [],
            }
            for item in self.inventory
        ]
        schema = {
            "structure_name": "string",
            "pieces": [
                {
                    "piece_id": "logical unique id",
                    "object_id": "must come from inventory",
                    "asset_name": "must match inventory",
                    "role": "base|wall|roof|bridge|decoration|generic",
                    "pose_class": "flat|vertical",
                    "anchor_offset_m": (
                        "[x,y,z] only for grounded flat pieces; otherwise null"
                    ),
                    "anchor_yaw_deg": 0.0,
                }
            ],
            "connections": [
                {
                    "connection_id": "unique id",
                    "joint_type": (
                        "flat_stack|right_angle_edge|vertical_slot|bridge"
                    ),
                    "child_id": "piece id",
                    "parent_ids": ["piece ids"],
                    "parent_edges": ["+x|-x|+y|-y"],
                    "child_edge": "-y",
                    "overlap_fraction": 0.5,
                    "gap_m": 0.002,
                    "clearance_m": 0.008,
                    "flip": False,
                }
            ],
        }
        return (
            "Compile the requested structure into a bounded magnetic-panel "
            "assembly graph.\n\n"
            f"User description: {description}\n"
            f"Maximum pieces: {self.max_pieces}\n"
            f"Inventory: {json.dumps(inventory, ensure_ascii=False)}\n"
            f"Output schema: {json.dumps(schema, ensure_ascii=False)}\n\n"
            "Hard physical rules:\n"
            "1. Use each inventory object at most once.\n"
            "2. A grounded anchor must be a lying/flat panel.\n"
            "3. A vertical panel may NOT stand on one lying panel. It must use "
            "vertical_slot with exactly two different lying supports.\n"
            "4. A 90-degree wall corner uses right_angle_edge and "
            "overlap_fraction=0.5; the parent must already be a stable vertical "
            "panel.\n"
            "5. A roof or bridge uses at least two stable supports.\n"
            "6. Keep gap_m in [0.0005, 0.015] and clearance_m in [0.004, 0.04].\n"
            "7. Do not emit poses or trajectories; only symbolic placements.\n"
            "8. Every non-anchor piece has exactly one connection.\n"
            "Return JSON only."
        )

    def _from_mapping(
        self,
        raw: Mapping[str, Any],
        *,
        anchor_pose: Sequence[Sequence[float]],
        description: str,
    ) -> MagneticAssemblySpec:
        pieces = tuple(
            MagneticPiece(
                piece_id=str(item["piece_id"]),
                object_id=str(item["object_id"]),
                asset_name=str(item["asset_name"]),
                role=PanelRole(str(item.get("role", "generic"))),
                pose_class=PanelPoseClass(str(item.get("pose_class", "flat"))),
                anchor_offset_m=(
                    None
                    if item.get("anchor_offset_m") is None
                    else tuple(float(value) for value in item["anchor_offset_m"])
                ),
                anchor_yaw_deg=float(item.get("anchor_yaw_deg", 0.0)),
                metadata=dict(item.get("metadata") or {}),
            )
            for item in raw.get("pieces") or ()
        )
        connections = tuple(
            MagneticConnection(
                connection_id=str(item["connection_id"]),
                joint_type=MagneticJointType(str(item["joint_type"])),
                child_id=str(item["child_id"]),
                parent_ids=tuple(
                    str(value) for value in item.get("parent_ids") or ()
                ),
                parent_edges=tuple(
                    PanelEdge(str(value))
                    for value in item.get("parent_edges") or ()
                ),
                child_edge=PanelEdge(str(item.get("child_edge", "-y"))),
                overlap_fraction=float(item.get("overlap_fraction", 0.5)),
                gap_m=float(item.get("gap_m", 0.002)),
                clearance_m=float(item.get("clearance_m", 0.008)),
                flip=bool(item.get("flip", False)),
                metadata=dict(item.get("metadata") or {}),
            )
            for item in raw.get("connections") or ()
        )
        return MagneticAssemblySpec(
            structure_name=str(
                raw.get("structure_name") or "generated_structure"
            ),
            pieces=pieces,
            connections=connections,
            anchor_pose=anchor_pose,
            description=description,
            max_pieces=self.max_pieces,
            metadata={
                "frontend": "llm_symbolic_graph",
                "raw_model_metadata": dict(raw.get("metadata") or {}),
            },
        )

    def generate(
        self,
        description: str,
        *,
        anchor_pose: Sequence[Sequence[float]],
        llm: StructureLLM | None = None,
        allow_template_fallback: bool = True,
    ) -> MagneticAssemblySpec:
        errors: list[str] = []
        if llm is not None:
            prompt = self._prompt(description)
            for attempt in range(2):
                try:
                    response = llm.complete(prompt)
                    structure = self._from_mapping(
                        _extract_json(response),
                        anchor_pose=anchor_pose,
                        description=description,
                    )
                    report = self.rules.validate(
                        structure,
                        self.catalog,
                        self.inventory,
                    )
                    if report.valid:
                        return structure
                    errors = [
                        f"{item.code}: {item.message}"
                        for item in report.violations
                    ]
                except Exception as exc:
                    errors = [f"{type(exc).__name__}: {exc}"]
                if attempt == 0:
                    prompt += (
                        "\n\nThe previous response was invalid. Correct every "
                        "issue and return the complete JSON again:\n- "
                        + "\n- ".join(errors)
                    )
        if not allow_template_fallback:
            raise ValueError(
                "LLM could not produce a valid structure: " + "; ".join(errors)
            )
        return self.template(
            description,
            anchor_pose=anchor_pose,
            diagnostics={"llm_errors": errors},
        )

    def _take(
        self,
        used: set[str],
        role: PanelRole,
        count: int,
    ) -> list[MagneticInventoryItem]:
        choices = [
            item
            for item in self.inventory
            if item.object_id not in used
            and item.asset_name in self.catalog
            and role in self.catalog[item.asset_name].allowed_roles
        ]
        if len(choices) < count:
            raise ValueError(
                f"inventory has only {len(choices)} unused {role.value} pieces, "
                f"need {count}"
            )
        selected = choices[:count]
        used.update(item.object_id for item in selected)
        return selected

    def template(
        self,
        description: str,
        *,
        anchor_pose: Sequence[Sequence[float]],
        diagnostics: Mapping[str, Any] | None = None,
    ) -> MagneticAssemblySpec:
        text = str(description).lower()
        if any(token in text for token in ("gate", "door", "门", "桥", "bridge")):
            kind = "gate"
        elif any(token in text for token in ("corner", "角", "l形", "l-shape")):
            kind = "corner"
        else:
            kind = "wall"
        used: set[str] = set()
        if kind == "wall":
            pieces, connections = self._wall_template(used)
        elif kind == "corner":
            pieces, connections = self._corner_template(used)
        else:
            pieces, connections = self._gate_template(used)
        structure = MagneticAssemblySpec(
            structure_name=f"template_{kind}",
            pieces=tuple(pieces),
            connections=tuple(connections),
            anchor_pose=anchor_pose,
            description=description,
            max_pieces=self.max_pieces,
            metadata={
                "frontend": "deterministic_template",
                **dict(diagnostics or {}),
            },
        )
        report = self.rules.validate(structure, self.catalog, self.inventory)
        if not report.valid:
            raise ValueError(
                "template is invalid: "
                + "; ".join(
                    f"{item.code}: {item.message}"
                    for item in report.violations
                )
            )
        return structure

    def _wall_template(
        self,
        used: set[str],
        *,
        prefix: str = "",
        x_offset: float = 0.0,
    ) -> tuple[list[MagneticPiece], list[MagneticConnection]]:
        bases = self._take(used, PanelRole.BASE, 2)
        wall = self._take(used, PanelRole.WALL, 1)[0]
        wall_spec = self.catalog[wall.asset_name]
        left_spec = self.catalog[bases[0].asset_name]
        right_spec = self.catalog[bases[1].asset_name]
        separation = (
            0.5 * (left_spec.size_xyz_m[1] + right_spec.size_xyz_m[1])
            + wall_spec.size_xyz_m[2]
            + 0.004
        )
        ids = {
            "left": f"{prefix}base_left",
            "right": f"{prefix}base_right",
            "wall": f"{prefix}wall",
        }
        pieces = [
            MagneticPiece(
                ids["left"],
                bases[0].object_id,
                bases[0].asset_name,
                PanelRole.BASE,
                PanelPoseClass.FLAT,
                (x_offset, -0.5 * separation, 0.0),
            ),
            MagneticPiece(
                ids["right"],
                bases[1].object_id,
                bases[1].asset_name,
                PanelRole.BASE,
                PanelPoseClass.FLAT,
                (x_offset, 0.5 * separation, 0.0),
            ),
            MagneticPiece(
                ids["wall"],
                wall.object_id,
                wall.asset_name,
                PanelRole.WALL,
                PanelPoseClass.VERTICAL,
            ),
        ]
        connections = [
            MagneticConnection(
                f"{prefix}slot",
                MagneticJointType.VERTICAL_SLOT,
                ids["wall"],
                (ids["left"], ids["right"]),
                (PanelEdge.POS_Y, PanelEdge.NEG_Y),
                gap_m=0.002,
                clearance_m=max(0.008, wall_spec.engagement_clearance_m),
            )
        ]
        return pieces, connections

    def _corner_template(
        self,
        used: set[str],
    ) -> tuple[list[MagneticPiece], list[MagneticConnection]]:
        pieces, connections = self._wall_template(used)
        second = self._take(used, PanelRole.WALL, 1)[0]
        pieces.append(
            MagneticPiece(
                "wall_corner",
                second.object_id,
                second.asset_name,
                PanelRole.WALL,
                PanelPoseClass.VERTICAL,
            )
        )
        connections.append(
            MagneticConnection(
                "corner_joint",
                MagneticJointType.RIGHT_ANGLE_EDGE,
                "wall_corner",
                ("wall",),
                (PanelEdge.POS_X,),
                child_edge=PanelEdge.NEG_X,
                overlap_fraction=0.5,
                gap_m=0.002,
                clearance_m=max(
                    0.008,
                    self.catalog[second.asset_name].engagement_clearance_m,
                ),
            )
        )
        return pieces, connections

    def _gate_template(
        self,
        used: set[str],
    ) -> tuple[list[MagneticPiece], list[MagneticConnection]]:
        left_pieces, left_connections = self._wall_template(
            used,
            prefix="left_",
            x_offset=-0.08,
        )
        right_pieces, right_connections = self._wall_template(
            used,
            prefix="right_",
            x_offset=0.08,
        )
        bridge = self._take(used, PanelRole.BRIDGE, 1)[0]
        pieces = left_pieces + right_pieces + [
            MagneticPiece(
                "top_bridge",
                bridge.object_id,
                bridge.asset_name,
                PanelRole.BRIDGE,
                PanelPoseClass.FLAT,
            )
        ]
        connections = left_connections + right_connections + [
            MagneticConnection(
                "top_bridge_joint",
                MagneticJointType.BRIDGE,
                "top_bridge",
                ("left_wall", "right_wall"),
                gap_m=0.002,
                clearance_m=max(
                    0.008,
                    self.catalog[bridge.asset_name].engagement_clearance_m,
                ),
            )
        ]
        if len(pieces) > self.max_pieces:
            raise ValueError("gate template exceeds the configured piece limit")
        return pieces, connections
