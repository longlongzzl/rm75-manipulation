#!/usr/bin/env python3
"""Separate safety/motion observations; never infer safety from target success."""
import argparse
import gzip
import json
from pathlib import Path

import numpy as np


def contact_metrics(contacts, object_id):
    penetration, lateral = 0., 0.
    object_name = f"scene-0_{object_id}"
    for contact in contacts:
        pair = (contact["body_a"], contact["body_b"])
        if object_name not in pair:
            continue
        gripper = any(name.startswith("gripper_") or name in ("left_pad", "right_pad") for name in pair)
        for point in contact["points"]:
            lateral += float(np.linalg.norm(point["impulse_ns"][:2]))
            if gripper:
                penetration = max(penetration, -float(point["separation_m"]))
    return penetration, lateral


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence-dir", type=Path, required=True)
    parser.add_argument("--compiled-dir", type=Path, required=True)
    args = parser.parse_args()
    evidence = json.loads((args.evidence_dir / "evidence_summary.json").read_text())
    frozen = json.loads((args.compiled_dir / "frozen_inputs.json").read_text())
    atoms = {a["atom_id"]: a for a in frozen["plan"]["atoms"]}
    names = evidence["joint_names"]
    gripper_indices = [i for i,n in enumerate(names) if n.startswith("gripper_")]
    report = {"state": "NEEDS_REVIEW", "categories": {}, "by_atom_phase": {},
              "shadow_readiness": evidence["shadow_readiness"], "frames": 0,
              "frame_sequence_contiguous": True,
              "interpretation": "Raw maxima, not calibrated acceptance thresholds; lateral impulse is not proof of causal side push; negative separation is retained, not silently treated as allowed contact"}
    previous = 0
    global_metrics = {}
    with gzip.open(args.evidence_dir / "physics_substeps.jsonl.gz", "rt") as stream:
        for line in stream:
            sample = json.loads(line)
            report["frames"] += 1
            report["frame_sequence_contiguous"] &= sample["frame_id"] == previous + 1
            previous = sample["frame_id"]
            atom = atoms[sample["atom_id"]]
            penetration, lateral = contact_metrics(sample["contacts"], atom["object_id"])
            obj = sample["objects"][atom["object_id"]]
            support = atom.get("support_object_id")
            drift = 0.
            if support:
                initial = np.array(frozen["scene"]["objects"][support]["pose"])
                actual = np.array(sample["objects"][support]["pose"])
                drift = float(np.linalg.norm(actual[:3,3]-initial[:3,3]))
            metrics = {"gripper_limit_excess_rad": max((sample["limit_excess_rad"][i] for i in gripper_indices), default=0.),
                       "all_joint_limit_excess_rad": max(sample["limit_excess_rad"]),
                       "gripper_object_penetration_m": penetration,
                       "arm_tracking_error_rad": max(abs(x) for x in sample["arm_tracking_error_rad"]),
                       "object_lateral_point_impulse_norm_sum_ns": lateral,
                       "support_translation_from_frozen_m": drift,
                       "object_linear_speed_m_s": float(np.linalg.norm(obj["linear_velocity_m_s"])),
                       "object_angular_speed_rad_s": float(np.linalg.norm(obj["angular_velocity_rad_s"]))}
            key = sample["atom_id"] + "/" + sample["phase"]
            group = report["by_atom_phase"].setdefault(key, {})
            for name,value in metrics.items():
                for dest in (group, global_metrics):
                    if name not in dest or value > dest[name]["value"]:
                        dest[name] = {"value": value, "frame_id": sample["frame_id"],
                                      "simulation_time_s": sample["simulation_time_s"]}
    report["categories"] = {
        "joint_limits": {"status": "excursion_observed" if global_metrics["gripper_limit_excess_rad"]["value"]>0 else "no_excursion_observed", "peak": global_metrics["gripper_limit_excess_rad"]},
        "gripper_object_penetration": {"status": "penetration_observed" if global_metrics["gripper_object_penetration_m"]["value"]>0 else "none_observed", "peak": global_metrics["gripper_object_penetration_m"]},
        "tracking": {"status": "unknown_acceptance_threshold", "peak": global_metrics["arm_tracking_error_rad"]},
        "contact_side_impulse": {"status": "observed_impulse_not_causal_proof", "peak": global_metrics["object_lateral_point_impulse_norm_sum_ns"]},
        "support_target_consistency": {"status": "unknown_calibrated_support_tolerance", "support_translation_peak": global_metrics["support_translation_from_frozen_m"], "note": "Support translation relative to frozen scene is measured separately; target/support geometry and rotation acceptance are not established by this metric"}}
    report["support_relative_release_observations"] = []
    for boundary in evidence["release_boundaries"]:
        atom = atoms[boundary["atom_id"]]
        support = atom.get("support_object_id")
        sample = boundary["observation"]
        if not support or sample is None:
            continue
        initial_support = np.array(frozen["scene"]["objects"][support]["pose"])
        observed_support = np.array(sample["objects"][support]["pose"])
        observed_object = np.array(sample["objects"][atom["object_id"]]["pose"])
        expected_relative = np.linalg.inv(initial_support) @ np.array(atom["target_pose"])
        observed_relative = np.linalg.inv(observed_support) @ observed_object
        report["support_relative_release_observations"].append({
            "atom_id": atom["atom_id"], "boundary": boundary["boundary"],
            "frame_id": sample["frame_id"], "simulation_time_s": sample["simulation_time_s"],
            "expected_object_in_frozen_support": expected_relative.tolist(),
            "observed_object_in_observed_support": observed_relative.tolist(),
            "relative_translation_error_m": float(np.linalg.norm(expected_relative[:3,3]-observed_relative[:3,3])),
            "acceptance": "unknown; raw relative pose, not a contact stability or symmetry-aware success validator"})
    (args.evidence_dir / "assessment.json").write_text(json.dumps(report, indent=2))
    print(json.dumps({"frames": report["frames"], "categories": report["categories"]}, indent=2))


if __name__ == "__main__":
    main()
