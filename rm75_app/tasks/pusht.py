from __future__ import annotations
from rm75_app.core.contracts import PipelineStage,TaskDefinition,TaskRequest
from rm75_app.tasks.base import TaskAdapterBase
from rm75_app.workcell.task_command import command


class PushTTask(TaskAdapterBase):
    definition=TaskDefinition(key='pusht',title='PushT 闭环推移',family='nonprehensile',
        description='新观测驱动的短程预测、碰撞校验、真机推移和再观测；不是 PPO。',
        capabilities=('closed_loop','frontend','realsense','curobo2','realman'),
        modes=('preview','sim','real'),default_mode='preview',aliases=('push-t',),
        stages=(PipelineStage.TASK_INPUT,PipelineStage.PERCEPTION,PipelineStage.MOTION_PLANNING,
                PipelineStage.EXECUTION,PipelineStage.VALIDATION),
        status='implemented_unverified_hardware',backend='rm75_app.pusht',
        notes=('sim is an explicitly labelled CPU surrogate, not a ManiSkill validation.',))
    def command(self,request:TaskRequest,*,python='python'):
        request=self.normalize_request(request)
        errors=self.validate_request(request)
        if errors:
            raise ValueError('; '.join(errors))
        return command(request,task='pusht',mode=request.mode or 'preview',python=python)
