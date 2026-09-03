#!/usr/bin/env python3
"""Audit attached-object collisions at carrot preplace IK endpoints."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from rm75_app.planning.backends.curobo2 import Curobo2Backend, Curobo2BackendConfig
from rm75_app.tasks.manipulation_plan import load_plan
from rm75_app.validation.curobo_gate import run_curobo_gate


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--max-endpoints", type=int, default=48)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    audit: list[dict[str, Any]] = []
    seen: set[str] = set()
    screened: dict[str, list[float]] = {}
    original_plan = Curobo2Backend.plan_candidates
    original_prepare = Curobo2Backend.prepare_pose_candidates_coarse

    def audited_prepare_pose_candidates_coarse(
        self: Curobo2Backend,
        candidates,
        scene,
        *,
        tool_frame="gripper_tcp",
        ignore_object_name=None,
        ignore_object_names=(),
    ):
        result = original_prepare(
            self,
            candidates,
            scene,
            tool_frame=tool_frame,
            ignore_object_name=ignore_object_name,
            ignore_object_names=ignore_object_names,
        )
        if not self._attachment_active:
            for candidate in candidates:
                candidate_id = str(candidate.candidate_id)
                if not candidate_id.startswith("preplace_"):
                    continue
                metrics = self._pose_ik_metrics.get(self._pose_cache_key(candidate))
                if metrics and bool(metrics.get("success", False)):
                    screened[candidate_id] = list(metrics["selected_joint_positions"])
        return result

    def audited_plan_candidates(self: Curobo2Backend, request):
        candidates = tuple(request.candidates)
        preplace = tuple(
            item
            for item in candidates
            if str(item.candidate_id).startswith("preplace_")
        )
        if (
            self._attachment_active
            and preplace
            and len(audit) < int(args.max_endpoints)
        ):
            fresh = tuple(
                item
                for item in preplace
                if item.candidate_id not in seen
                and str(item.candidate_id) in screened
            )[: int(args.max_endpoints) - len(audit)]
            if fresh:
                planner = self._ensure_planner()
                modules = self._import_modules()
                torch = modules["torch"]
                current_state = modules["JointState"].from_position(
                    torch.as_tensor(
                        request.current.positions,
                        dtype=planner.device_cfg.dtype,
                        device=planner.device_cfg.device,
                    ).reshape(1, -1),
                    joint_names=list(request.current.names),
                )
                for candidate in fresh:
                    seen.add(str(candidate.candidate_id))
                    target_positions = np.asarray(
                        screened[str(candidate.candidate_id)], dtype=np.float64
                    )
                    target_state = modules["JointState"].from_position(
                        torch.as_tensor(
                            target_positions,
                            dtype=planner.device_cfg.dtype,
                            device=planner.device_cfg.device,
                        ).reshape(1, -1),
                        joint_names=list(request.current.names),
                    )
                    contacts = self._collision_diagnostics_for_states(
                        planner,
                        {"lift_start": current_state, "preplace_target": target_state},
                        ignored_world_objects={"carriot"},
                    )
                    audit.append(
                        {
                            "candidate_id": candidate.candidate_id,
                            "coarse_ik_found": True,
                            "target_joint_positions": target_positions.tolist(),
                            "contacts": contacts,
                        }
                    )
        return original_plan(self, request)

    Curobo2Backend.prepare_pose_candidates_coarse = audited_prepare_pose_candidates_coarse
    Curobo2Backend.plan_candidates = audited_plan_candidates
    try:
        plan = load_plan(args.plan)
        gate = run_curobo_gate(
            plan,
            output,
            backend_config=Curobo2BackendConfig(
                max_batch_size=4,
                max_goalset=4,
                num_ik_seeds=64,
                num_trajopt_seeds=4,
                attachment_num_spheres=64,
            ),
            max_attempts_per_atom=1,
        )
    finally:
        Curobo2Backend.plan_candidates = original_plan
        Curobo2Backend.prepare_pose_candidates_coarse = original_prepare
    payload = {
        "gate_status": gate.status.value,
        "gate_summary": gate.summary,
        "audited_endpoint_count": len(audit),
        "screened_preplace_ik_count": len(screened),
        "coarse_ik_count": sum(bool(item["coarse_ik_found"]) for item in audit),
        "collision_free_target_count": sum(
            bool(item["coarse_ik_found"])
            and not any(
                contact.get("state") == "preplace_target"
                for contact in item["contacts"]
            )
            for item in audit
        ),
        "records": audit,
    }
    result_file = output / "preplace_endpoint_audit.json"
    result_file.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
