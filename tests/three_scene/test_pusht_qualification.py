import copy
import json
from pathlib import Path
from rm75_app.pusht.qualification import planning_qualification
from rm75_app.pusht.observation import Observation
import pytest
import time


def test_example_profile_remains_unqualified_and_is_not_modified():
    root=Path(__file__).resolve().parents[2]
    profile=json.loads((root/'examples/workcell/machine.example.json').read_text())
    before=copy.deepcopy(profile)
    result=planning_qualification(profile)
    assert not result['planning_inputs_complete'] and not result['motion_authorized']
    fields={row['field'] for row in result['errors']}
    assert {'pusht.motion.push_tcp_z_m','pusht.motion.pusher_contact_links',
            'pusht.observer.T_base_camera','pusht.observer.T_marker_object'} <= fields
    assert profile==before


@pytest.mark.parametrize('delta',[-.1,0.])
def test_sequence_increment_cannot_mask_timestamp_regression(delta):
    now=time.time()
    old=Observation('s',1,now,(.35,0,0),'live_tracker')
    new=Observation('s',2,now+delta,(.35,0,0),'live_tracker')
    with pytest.raises(ValueError,match='non_monotonic'):
        new.validate(previous=old,real=True)
