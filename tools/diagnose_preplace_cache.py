#!/usr/bin/env python3
"""Audit preplace IK-cache state after attaching the held object."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from rm75_app.planning.backends.curobo2 import Curobo2Backend
from rm75_app.tasks.manipulation_plan import load_plan
from rm75_app.validation.curobo_gate import run_curobo_gate


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    records: list[dict[str, object]] = []
    original = Curobo2Backend.plan_candidates

    def audited(self: Curobo2Backend, request):
        candidates = tuple(request.candidates)
        if self._attachment_active and any(
            str(item.candidate_id).startswith("preplace_") for item in candidates
        ):
            current_hits = 0
            opposite_hits = 0
            items = []
            for candidate in candidates:
                key = self._pose_cache_key(candidate)
                opposite = (*key[:-1], 1.0 - float(key[-1]))
                current_hit = key in self._pose_ik_cache
                opposite_hit = opposite in self._pose_ik_cache
                current_hits += int(current_hit)
                opposite_hits += int(opposite_hit)
                items.append(
                    {
                        "candidate_id": candidate.candidate_id,
                        "current_hit": current_hit,
                        "opposite_hit": opposite_hit,
                    }
                )
            records.append(
                {
                    "gripper_state": self._gripper_collision_state,
                    "current_hits": current_hits,
                    "opposite_hits": opposite_hits,
                    "items": items,
                }
            )
        return original(self, request)

    Curobo2Backend.plan_candidates = audited
    try:
        result = run_curobo_gate(load_plan(args.plan), args.output_dir)
    finally:
        Curobo2Backend.plan_candidates = original
    payload = {
        "gate": result.as_dict(),
        "preplace_call_count": len(records),
        "current_hit_count": sum(int(item["current_hits"]) for item in records),
        "opposite_hit_count": sum(int(item["opposite_hits"]) for item in records),
        "records": records,
    }
    output = args.output_dir.expanduser().resolve() / "preplace_cache_audit.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in payload.items() if key != "records"}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
