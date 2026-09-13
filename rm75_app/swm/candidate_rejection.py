"""Expected physical candidate infeasibility, never corrupt input or cancellation."""
from .scene import SceneInvalid


class CandidateInfeasible(SceneInvalid):
    def __init__(self,reason,*,stage,sample=None):
        super().__init__(reason)
        self.stage=stage;self.sample=sample
