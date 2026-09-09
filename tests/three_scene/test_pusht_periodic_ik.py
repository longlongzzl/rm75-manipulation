from types import SimpleNamespace as NS
import numpy as np
from rm75_app.planning.backends.curobo2 import Curobo2Backend
from rm75_app.planning.contracts import JointConfiguration


class Tensor:
    def __init__(self,data):self.data=np.asarray(data)
    def __getitem__(self,index):return Tensor(self.data[index])
    def reshape(self,*shape):return Tensor(self.data.reshape(*shape))
    def detach(self):return self
    def cpu(self):return self
    def contiguous(self):return self
    def numpy(self):return self.data


def test_stage_ik_keeps_all_success_rows_and_legal_periodic_equivalents():
    backend=Curobo2Backend.__new__(Curobo2Backend);scene=object();backend._scene=scene
    backend.config=NS(coarse_ik_return_seeds=12)
    raw=np.array([[.4,-4.1],[-.3,5.8],[.2,0.]])
    ik=NS(reset_seed=lambda:None,solve_pose=lambda *a,**k:NS(solution=Tensor(raw),success=Tensor([True,True,False])))
    planner=NS(ik_solver=ik,kinematics=NS(get_joint_limits=lambda:NS(position=Tensor([[-3.1,-6.28],[3.1,6.28]]))))
    backend._ensure_planner=lambda:planner
    backend._make_batch_inputs=lambda request:(NS(position=Tensor([[0.,1.047]])),object())
    result=backend.solve_pose_ik_variants(NS(candidates=[object()],scene=scene,
        current=JointConfiguration(('bounded','periodic'),[0.,1.047])))
    assert len(result)==2
    np.testing.assert_allclose([q.positions for q in result],[[.4,-4.1+2*np.pi],[-.3,5.8-2*np.pi]])
    np.testing.assert_allclose(raw,[[.4,-4.1],[-.3,5.8],[.2,0.]])
    assert backend._stage_ik_periodic_audit==dict(successful_rows=2,retained_rows=2,wrapped_rows=2,wrapped_joints=['periodic'])
