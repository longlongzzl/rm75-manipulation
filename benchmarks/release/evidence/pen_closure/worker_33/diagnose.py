"""Full original private scene closure prediction; not a skill release gate."""
import copy,json,sys,time
from pathlib import Path
from contextlib import ExitStack
from dataclasses import replace
import numpy as np
ROOT=Path('/home/zhangzhao/Desktop/rm75-manipulation')
sys.path.insert(0,str(ROOT))
from rm75_app.swm.native_execution import NativePrimaryExecutor
from rm75_app.swm.native_robot import read_primary_drive_state
from rm75_app.swm.native_robot_mirror import SapienRobotStatePort,apply_native_drive_state
from rm75_app.swm.native_body_mirror import NativeBodyMirror,native_pose
from rm75_app.swm.native_scene import SapienScenePort
from rm75_app.swm.native_registration import file_digest
from rm75_app.swm.native_bootstrap import _array
from rm75_app.swm.scene import SceneWorldModel,SyncPolicy,SceneInvalid
registration=None
resolved_urdf=None
original_body=NativeBodyMirror.__init__
def remember_body(self,reg,robot_port,*,resources):
    global registration
    original_body(self,reg,robot_port,resources=resources)
    if registration is None:registration=reg
NativeBodyMirror.__init__=remember_body
original_robot=SapienRobotStatePort.__init__
def remember_robot(self,*args,**kwargs):
    global resolved_urdf
    original_robot(self,*args,**kwargs)
    if resolved_urdf is None:resolved_urdf=self.urdf_path
SapienRobotStatePort.__init__=remember_robot
original_execute=NativePrimaryExecutor.execute_trajectory
ran=False
def execute(self,stage,trajectory):
    global ran
    result=original_execute(self,stage,trajectory)
    if stage!='grasp' or ran:return result
    ran=True
    primary=self.primary
    report={'domain':'experimental_full_original_private_CPU_PhysX_closure',
        'skill_qualified':False,'model_qualified':False,'hardware_connected':False,'phase':'capture','steps':[]}
    output=ROOT/'runtime_data/swm_release_pen_worker_33/full_scene_probe.json'
    try:
        path=Path(registration.world.assets['bi']['mesh_path']).parent/'manifest.json'
        if file_digest(path)!=registration.evidence['manifest_sha256']:
            raise SceneInvalid('Original registered manifest changed')
        world=SceneWorldModel(json.loads(path.read_text()))
        world.check_assets()
        after=time.monotonic()
        batch=registration.source.capture(tuple(registration.source.bindings),after=after,
            boundary='diagnostic_preclose_full_scene')
        prepared=world.prepare_checkpoint(batch,after=after,now=time.monotonic(),
            policy=SyncPolicy(),boundary='diagnostic_preclose_full_scene')
        snapshot=world.commit_checkpoint(prepared)
        report['snapshot']=snapshot
        before=primary.read_state()
        source_drives=read_primary_drive_state(primary)
        report['phase']='construct_private_world'
        with ExitStack() as resources:
            robot=SapienRobotStatePort(resolved_urdf,resources=resources)
            robot.synchronize_primary_physics(primary)
            bodies=NativeBodyMirror(replace(registration,world=world),robot,resources=resources)
            port=SapienScenePort(bodies.actors,asset_bindings=bodies.asset_bindings,
                set_attachment=bodies.set_attachment,read_attachment=bodies.read_attachment,
                apply_physics=bodies.apply_physics,read_physics=bodies.read_physics,
                make_pose=native_pose,robot_port=robot,body_port=bodies)
            port.apply_idle_snapshot(snapshot)
            report['object_ids']=sorted(bodies.actors)
            report['initial_robot_ack']=robot.acknowledgement
            report['initial_body_ack']=bodies.readback()
            controller=primary.env.unwrapped.agent.controller.controllers['gripper']
            cfg=controller.config
            if cfg.use_delta or cfg.use_target or cfg.interpolate:
                raise SceneInvalid('Closure probe requires original absolute noninterpolating mimic control')
            import torch
            command=controller._preprocess_action(torch.ones((1,controller.effective_dof),device=controller.device))
            targets=torch.zeros_like(controller._target_qpos)
            targets[:,controller.control_joint_indices]=command
            targets[:,controller.mimic_joint_indices]=(targets[:,controller.mimic_control_joint_indices]
                *controller._multiplier[None,:]+controller._offset[None,:])
            closed=copy.deepcopy(source_drives)
            for joint,value in zip(controller.joints,_array(targets).reshape(-1)):
                closed['position_targets'][closed['joint_names'].index(joint.name)]=float(value)
            report['closed_drive_targets']=closed
            report['phase']='private_closure_steps'
            links=robot._robot.get_links()
            robot_entities=[link.entity for link in links]
            from mani_skill.utils.sapien_utils import compute_total_impulse
            dt=closed['physics_timestep_s']
            count=round(closed['simulation_frequency_hz']/closed['control_frequency_hz'])
            for tick in range(20):
                apply_native_drive_state(robot._robot,robot._physics_system,closed)
                for substep in range(count):
                    primary.stop.check()
                    robot._scene.step()
                    contacts=[]
                    for contact in robot._physics_system.get_contacts():
                        if not any(body.entity in robot_entities for body in contact.bodies):continue
                        contacts.append(dict(body_names=[body.entity.name for body in contact.bodies],
                            force_in_base_n=(compute_total_impulse([(contact,True)])/dt).tolist(),
                            separations_m=[getattr(point,'separation',None) for point in contact.points]))
                    report['steps'].append(dict(control_tick=tick+1,substep=substep+1,
                        q=_array(robot._robot.get_qpos()).reshape(-1).tolist(),
                        qdot=_array(robot._robot.get_qvel()).reshape(-1).tolist(),contacts=contacts))
            report['phase']='ownership_check'
            final=primary.read_state()
            unchanged=all(final[key]==before[key] for key in ('positions','velocities','objects'))
            unchanged=unchanged and read_primary_drive_state(primary)==source_drives
            report['primary_unchanged_while_private_stepped']=unchanged
            if not unchanged:raise SceneInvalid('Private prediction changed primary state')
        report['private_robot_closed']=robot.closed
        report['phase']='complete_unqualified_prediction'
    except Exception as error:
        report['error']=repr(error)
        raise
    finally:
        output.write_text(json.dumps(report,indent=2))
    return result
NativePrimaryExecutor.execute_trajectory=execute
from rm75_app.workcell.worker import main
raise SystemExit(main())
