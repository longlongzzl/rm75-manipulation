from tools.predict_swm_future_push import compact_prediction_result
from rm75_app.swm.scene import digest


def test_dense_prediction_summary_keeps_digest_not_full_large_trace():
    result=dict(time_s=[0.,1.,2.],T_world_object=['initial','middle','final'],
                predicted_tool_readback={'large':'trace'},valid=True)
    compact=compact_prediction_result(result)
    assert compact['prediction_samples']==3 and compact['time_s']==[0.,2.]
    assert compact['T_world_object']==['initial','final']
    assert compact['trajectory_digest']==digest(dict(time_s=result['time_s'],T_world_object=result['T_world_object']))
    assert 'predicted_tool_readback' not in compact
