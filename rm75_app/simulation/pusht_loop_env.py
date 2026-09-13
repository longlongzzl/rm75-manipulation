"""Live physics state for repeated replanned pushes, including an articulated RM75.

The full-arm branch loads the unchanged URDF, never a duplicate kinematic tool.
Drive constants and four-bar anchors come from the migrated RM75 agent, not Panda.
"""
import numpy as np
import sapien
from transforms3d.quaternions import mat2quat
from mani_skill.utils.registration import register_env
from .pusht_contact_env import PushTContactEnv
from rm75_app.pusht.physics_replay import forbidden_target_contact


@register_env('RM75-PushT-Loop-v1',max_episode_steps=1000000)
class PushTLoopEnv(PushTContactEnv):
    def __init__(self,*args,full_arm=False,urdf=None,gripper_locks=None,gravity_compensation='none',**kwargs):
        if gravity_compensation not in ('none','original-agent'):raise ValueError('Unknown gravity model')
        self.gravity_compensation=gravity_compensation
        self.full_arm=full_arm;self.urdf=urdf;self.gripper_locks=gripper_locks or {}
        self.active_program=None;self.program_started=0.;self.tracking=[]
        self.self_contact_pairs={};self.static_contact_pairs=[];self.tcp_tracking=[];self.qvel_peak=0.
        self.forbidden_target_contacts=[]
        super().__init__(*args,**kwargs)

    def _load_tool(self,material,tool_visual):
        if not self.full_arm:return super()._load_tool(material,tool_visual)
        loader=self.scene.create_urdf_loader();loader.fix_root_link=True;loader.disable_self_collisions=False
        loader.load_multiple_collisions_from_file=True
        self.robot=loader.load(str(self.urdf))
        if self.gravity_compensation=='original-agent':
            # Exact BaseAgent.set_control_mode balance_passive_force=True
            # convention. Ideal arm-only compensation, NOT a torque-controller
            # qualification. Dynamic T and all other bodies retain gravity.
            for link in self.robot.get_links():link.disable_gravity=True
        self.joints=self.robot.get_active_joints();self.joint_names=[joint.name for joint in self.joints]
        self.arm_indices=[self.joint_names.index(f'joint_{i}') for i in range(1,8)]
        self.gripper_indices={name:self.joint_names.index(name) for name in self.gripper_locks}
        self.tool_links=[link for link in self.robot.get_links()
                         if link.name.startswith('gripper_') or link.name in ('left_pad','right_pad')]
        self.tcp=self.robot.links_map['gripper_tcp']
        self.robot_entities={link._bodies[0].entity:link.name for link in self.robot.get_links()}
        for joint in self.joints:
            if joint.name.startswith('joint_'):
                joint.set_drive_properties(stiffness=1000.,damping=100.,force_limit=20.)
                joint.set_friction(.1)
            elif joint.name in ('gripper_Right_1_Joint','gripper_Left_1_Joint'):
                joint.set_drive_properties(stiffness=1000.,damping=100.,force_limit=5.)
                joint.set_friction(1.)
            else:
                joint.set_drive_properties(stiffness=0.,damping=0.,force_limit=0.)
                joint.set_friction(0.)
        for side in ('Left','Right'):
            drive=self.scene.create_drive(self.robot.links_map[f'gripper_{side}_2_Link'],
                sapien.Pose([.0404,.0375,0]),self.robot.links_map[f'gripper_{side}_Support_Link'],
                sapien.Pose([-.0141,.017,0]))
            drive.set_limit_x(0,0);drive.set_limit_y(0,0);drive.set_limit_z(0,0)
        # No blanket self-collision disable or gripper/link6/link7 group masks.

    def _initialize_episode(self,env_idx,options):
        super()._initialize_episode(env_idx,options)
        self.commanded_q=self.program.initial.copy();self.active_program=None;self.tracking=[]
        if self.full_arm:
            q=np.zeros(len(self.joints));q[self.arm_indices]=self.program.initial
            for name,index in self.gripper_indices.items():q[index]=self.gripper_locks[name]
            self.robot.set_qpos(q);self.robot.set_qvel(np.zeros_like(q))
            self._drive(q[self.arm_indices])

    def _drive(self,q):
        for index,value in zip(self.arm_indices,q):self.joints[index].set_drive_target(float(value))
        for name in ('gripper_Right_1_Joint','gripper_Left_1_Joint'):
            self.joints[self.joint_names.index(name)].set_drive_target(float(self.gripper_locks[name]))

    def read_q(self):
        if not self.full_arm:return self.commanded_q.copy()
        return self.robot.get_qpos().detach().cpu().numpy().reshape(-1)[self.arm_indices].copy()

    def _before_simulation_step(self):
        self.physics_time+=1/self.sim_freq
        if self.active_program is not None:
            elapsed=self.physics_time-self.program_started
            self.stage,self.commanded_q=self.active_program.configuration(elapsed)
            if elapsed>self.active_program.duration:
                self.stage=getattr(self.active_program,'hold_stage','post_settle')
        else:self.stage='settle'
        if self.full_arm:self._drive(self.commanded_q)
        else:
            transform=self.program.fk(self.commanded_q)
            self.tool_body.set_kinematic_target(sapien.Pose(transform[:3,3],mat2quat(transform[:3,:3])))

    def _after_simulation_step(self):
        if not self.full_arm:return super()._after_simulation_step()
        impulse=sum(float(np.linalg.norm(self.scene.get_pairwise_contact_impulses(link,self.target).cpu().numpy()))
                    for link in self.tool_links)
        if impulse>0:
            self.contacts.append(dict(time_s=self.physics_time,stage=self.stage,impulse_Ns=impulse))
            if self.stage in ('settle','approach','descend'):self.early_contact_steps+=1
        for link in self.robot.get_links():
            for obj in self.world:
                contact_impulse=float(np.linalg.norm(self.scene.get_pairwise_contact_impulses(link,obj).cpu().numpy()))
                if contact_impulse>0:
                    self.obstacle_contact_steps+=1
                    self.static_contact_pairs.append(dict(time_s=self.physics_time,stage=self.stage,
                        link=link.name,object=obj.name,impulse_Ns=contact_impulse))
        for contact in self.scene.get_contacts():
            a,b=(body.entity for body in contact.bodies)
            target_entity=self.target._bodies[0].entity
            robot_entity=b if a==target_entity else a if b==target_entity else None
            if robot_entity in self.robot_entities:
                link_name=self.robot_entities[robot_entity]
                target_impulse=float(np.linalg.norm(np.sum([point.impulse for point in contact.points],axis=0)))
                if target_impulse>0 and forbidden_target_contact(link_name,self.stage,self.motion['pusher_contact_links']):
                    self.forbidden_target_contacts.append(dict(time_s=self.physics_time,stage=self.stage,
                        link=link_name,impulse_Ns=target_impulse))
            if a not in self.robot_entities or b not in self.robot_entities:continue
            impulse=float(np.linalg.norm(np.sum([point.impulse for point in contact.points],axis=0)))
            if impulse<=0:continue
            key=' / '.join(sorted([self.robot_entities[a],self.robot_entities[b]]))
            row=self.self_contact_pairs.setdefault(key,dict(substeps=0,max_impulse_Ns=0.))
            row['substeps']+=1;row['max_impulse_Ns']=max(row['max_impulse_Ns'],impulse)
        actual=self.tcp.pose.to_transformation_matrix().detach().cpu().numpy().reshape(4,4)
        desired=self.program.fk(self.commanded_q)
        angle=float(np.arccos(np.clip((np.trace(actual[:3,:3].T@desired[:3,:3])-1)/2,-1,1)))
        self.tcp_tracking.append((float(np.linalg.norm(actual[:3,3]-desired[:3,3])),angle))
        self.qvel_peak=max(self.qvel_peak,float(np.max(abs(self.robot.get_qvel().detach().cpu().numpy()))))
        self.tracking.append(float(np.max(abs(self.read_q()-self.commanded_q))))
