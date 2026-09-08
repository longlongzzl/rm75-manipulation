import json
from pathlib import Path
import numpy as np
import pytest
import trimesh
from tools.audit_jimu_saved_lift_height import audit,ROOT


def test_original_mesh_distance_and_height_sensitivity_are_read_only(tmp_path):
    mesh=ROOT/'rm75_app/_vendor/working_snapshot/Demo_Triangle/red_triangle_74x135x6p5.glb'
    before=mesh.read_bytes()
    geometry=trimesh.load(mesh,force='scene').to_geometry()
    apex=geometry.vertices[np.argmax(geometry.vertices[:,2])]
    row=dict(event='jimu_read_only_collision_diagnostic',step_id='unit_after_lift',
             world_objects=[dict(name='scene_obstacle_right_roof_triangle',file_path=str(mesh),
                                 scale=[1,1,1],pose=[0,0,0,1,0,0,0])],
             curobo_raw_world_collision=dict(nonzero=[dict(link='finger',center=(apex+[0,0,.005]).tolist(),radius=.01)]))
    source=tmp_path/'events.jsonl';source.write_text(json.dumps({'evidence':row})+'\n')
    raw=source.read_bytes();result=audit(source)
    assert source.read_bytes()==raw and mesh.read_bytes()==before
    clear=[row['sphere_mesh_clearance_m'] for row in result['rows']]
    np.testing.assert_allclose(clear,[-.005,0.,.005,.015],atol=1e-7)
    assert not result['new_gpu_planning'] and not result['full_robot_qualified']


def test_unknown_model_is_not_used_as_approved_triangle(tmp_path):
    path=ROOT/'rm75_app/_vendor/working_snapshot/pick_jiaobang/meshs/tennis_sim.glb'
    row=dict(event='jimu_read_only_collision_diagnostic',world_objects=[dict(
        name='scene_obstacle_right_roof_triangle',file_path=str(path))])
    source=tmp_path/'events.jsonl';source.write_text(json.dumps({'evidence':row})+'\n')
    with pytest.raises(ValueError,match='approved original triangle'):audit(source)
