import hashlib
import json
import numpy as np
import pytest
from rm75_app.swm.physics_replay import compound_cuboid_parts

def asset(tmp_path,parts):
    path=tmp_path/'compound.json'
    path.write_text(json.dumps(dict(schema='rm75_compound_cuboids_v1',units='m',parts=parts)))
    return dict(collision_path=str(path),collision_sha256=hashlib.sha256(path.read_bytes()).hexdigest())

def test_distinct_native_parts_not_convex_hull(tmp_path):
    a=np.eye(4);b=np.eye(4);b[1,3]=-.05
    parts=compound_cuboid_parts(asset(tmp_path,[dict(dimensions_m=[.1,.03,.02],T_collision_part=a.tolist()),
        dict(dimensions_m=[.03,.07,.02],T_collision_part=b.tolist())]))
    assert len(parts)==2 and parts[1][0][1,3]==-.05
    assert sum(np.prod(dims) for _,dims in parts)==pytest.approx(.000102)

@pytest.mark.parametrize('dims',[[0,.03,.02],[-1,1,1],[1,2],[float('nan'),1,1]])
def test_invalid_parts_reject(tmp_path,dims):
    with pytest.raises(ValueError):
        compound_cuboid_parts(asset(tmp_path,[dict(dimensions_m=dims,T_collision_part=np.eye(4).tolist())]))

def test_changed_descriptor_rejects(tmp_path):
    value=asset(tmp_path,[]);value['collision_sha256']='0'*64
    with pytest.raises(ValueError,match='Changed'):compound_cuboid_parts(value)
