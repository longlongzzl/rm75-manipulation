"""ManiSkill/PhysX contact dynamics: unchanged full tool spheres, dynamic T.

An isolated kinematic-tool test, NOT an articulated-arm servo simulation.
Only the tool receives kinematic targets. The T is moved by physics alone.
"""
import numpy as np
import sapien
import torch
from transforms3d.quaternions import mat2quat

from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.sensors.camera import CameraConfig
from mani_skill.utils import sapien_utils
from mani_skill.utils.registration import register_env
from mani_skill.utils.structs.types import SimConfig

from rm75_app.pusht.model import rectangles


@register_env('RM75-PushT-Contact-v1', max_episode_steps=10000)
class PushTContactEnv(BaseEnv):
    SUPPORTED_ROBOTS = ['none']

    def __init__(self, *args, program, spheres, config, observation, motion, static_objects,
                 stationary=False, **kwargs):
        self.program, self.spheres, self.config = program, np.asarray(spheres), config
        self.observation, self.motion, self.static_objects = observation, motion, static_objects
        self.stationary = stationary
        self.physics_time = 0.; self.settle_s = 2.; self.stage = 'settle'
        self.contacts = []; self.early_contact_steps = 0; self.obstacle_contact_steps = 0
        super().__init__(*args, robot_uids='none', **kwargs)

    @property
    def _default_sim_config(self):
        return SimConfig(sim_freq=240, control_freq=30)

    @property
    def _default_sensor_configs(self): return []

    @property
    def _default_human_render_camera_configs(self):
        center = np.array([.35, -.18, .02])
        return [CameraConfig('overview', sapien_utils.look_at(center+[.42, -.48, .42], center),
                             512, 512, .95, .01, 100),
                CameraConfig('side', sapien_utils.look_at(center+[-.03, -.42, .19], center),
                             512, 512, .9, .01, 100)]

    def _load_scene(self, options):
        material = sapien.physx.PhysxMaterial(.3, .3, 0.)
        table_visual = sapien.render.RenderMaterial(base_color=[.45, .42, .36, 1])
        target_visual = sapien.render.RenderMaterial(base_color=[.1, .75, .35, 1])
        tool_visual = sapien.render.RenderMaterial(base_color=[.2, .5, .95, 1])
        self.scene.set_ambient_light([.5, .5, .5])
        self.scene.add_directional_light([0, 0, -1], [1.5, 1.5, 1.5], shadow=True)
        self.world = []
        for obj in self.static_objects:
            builder = self.scene.create_actor_builder()
            builder.initial_pose = sapien.Pose(obj.pose.position, obj.pose.quaternion_wxyz)
            builder.add_box_collision(half_size=np.asarray(obj.dimensions)/2, material=material)
            builder.add_box_visual(half_size=np.asarray(obj.dimensions)/2, material=table_visual)
            self.world.append(builder.build_static(name=obj.name))
        yaw = self.observation.pose[2]
        self.target_pose = sapien.Pose([*self.observation.pose[:2], self.motion['object_centroid_z_m']],
                                     [np.cos(yaw/2), 0, 0, np.sin(yaw/2)])
        builder = self.scene.create_actor_builder(); builder.initial_pose = self.target_pose
        for x, y, w, h in rectangles(self.config):
            pose = sapien.Pose([x, y, 0]); size = [w/2, h/2, self.motion['object_height_m']/2]
            builder.add_box_collision(pose=pose, half_size=size, material=material, density=1000.)
            builder.add_box_visual(pose=pose, half_size=size, material=target_visual)
        self.target = builder.build(name='dynamic_T')
        self._load_tool(material, tool_visual)

    def _load_tool(self, material, tool_visual):
        transform = self.program.fk(self.program.initial)
        builder = self.scene.create_actor_builder()
        builder.initial_pose = sapien.Pose(transform[:3, 3], mat2quat(transform[:3, :3]))
        for x, y, z, radius in self.spheres:
            pose = sapien.Pose([x, y, z])
            builder.add_sphere_collision(pose=pose, radius=float(radius), material=material)
            builder.add_sphere_visual(pose=pose, radius=float(radius), material=tool_visual)
        self.tool = builder.build_kinematic(name='closed_gripper_original_38_spheres')
        self.tool_body = self.tool._objs[0].find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)

    def _initialize_episode(self, env_idx, options):
        self.physics_time = 0.; self.stage = 'settle'; self.contacts = []
        self.early_contact_steps = self.obstacle_contact_steps = 0
        # Initialization only. Never update the T pose during stepping/replay.
        self.target.set_pose(self.target_pose)

    def _before_simulation_step(self):
        self.physics_time += 1/self.sim_freq
        t = self.physics_time-self.settle_s
        self.stage, transform = self.program.sample(0. if self.stationary else max(0., t))
        if t < 0: self.stage = 'settle'
        elif t > self.program.duration: self.stage = 'post_settle'
        self.tool_body.set_kinematic_target(sapien.Pose(transform[:3, 3], mat2quat(transform[:3, :3])))

    def _after_simulation_step(self):
        impulse = float(torch.linalg.norm(self.scene.get_pairwise_contact_impulses(self.tool, self.target)))
        if impulse > 0:
            self.contacts.append(dict(time_s=self.physics_time, stage=self.stage, impulse_Ns=impulse))
            if self.stage in ('settle', 'approach', 'descend'): self.early_contact_steps += 1
        for obj in self.world:
            if float(torch.linalg.norm(self.scene.get_pairwise_contact_impulses(self.tool, obj))) > 0:
                self.obstacle_contact_steps += 1

    def _get_obs_extra(self, info): return {}

    def evaluate(self): return {}

    def get_state_dict(self):
        # ManiSkill 3.0.0b22's BaseEnv assumes a controller even for robot_uids='none'.
        # This environment has only rigid actors; preserve all their physical state.
        return self.scene.get_sim_state()
