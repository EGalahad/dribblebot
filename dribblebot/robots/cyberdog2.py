from .robot import Robot

from dribblebot import MINI_GYM_ROOT_DIR
import os
import numpy as np

from isaacgym import gymapi

class Cyberdog2(Robot):
    def __init__(self, env):
        super().__init__(env)

    def initialize(self):
        asset_file = '{MINI_GYM_ROOT_DIR}/resources/robots/cyberdog2/urdf/cyberdog2_v2.urdf'
        asset_path = asset_file.format(MINI_GYM_ROOT_DIR=MINI_GYM_ROOT_DIR)
        asset_root = os.path.dirname(asset_path)
        asset_file = os.path.basename(asset_path)

        asset_config = self.env.cfg.asset

        asset_options = gymapi.AssetOptions()
        asset_options.default_dof_drive_mode = asset_config.default_dof_drive_mode
        asset_options.collapse_fixed_joints = asset_config.collapse_fixed_joints
        asset_options.replace_cylinder_with_capsule = asset_config.replace_cylinder_with_capsule
        asset_options.flip_visual_attachments = asset_config.flip_visual_attachments
        asset_options.fix_base_link = asset_config.fix_base_link
        asset_options.density = asset_config.density
        asset_options.angular_damping = asset_config.angular_damping
        asset_options.linear_damping = asset_config.linear_damping
        asset_options.max_angular_velocity = asset_config.max_angular_velocity
        asset_options.max_linear_velocity = asset_config.max_linear_velocity
        asset_options.armature = asset_config.armature
        asset_options.thickness = asset_config.thickness
        asset_options.disable_gravity = asset_config.disable_gravity

        asset_options.vhacd_enabled = True
        asset_options.vhacd_params = gymapi.VhacdParams()
        asset_options.vhacd_params.resolution = 500000

        asset = self.env.gym.load_asset(self.env.sim, asset_root, asset_file, asset_options)

        self.num_dof = self.env.gym.get_asset_dof_count(asset)
        self.num_actuated_dof = 12
        self.num_bodies = self.env.gym.get_asset_rigid_body_count(asset)
        dof_props_asset = self.env.gym.get_asset_dof_properties(asset)
        rigid_shape_props_asset = self.env.gym.get_asset_rigid_shape_properties(asset)

        # check the props contains friction and damping, change them to randomly sampled from the range.
        joint_friction_range = self.env.cfg.domain_rand.joint_friction_range
        joint_damping_range = self.env.cfg.domain_rand.joint_damping_range
        for dof_prop in dof_props_asset:
            # friction originally 0.2
            # damping originally 0.05
            dof_prop['friction'] = np.random.uniform(joint_damping_range[0], joint_damping_range[1])
            dof_prop['damping'] = np.random.uniform(joint_friction_range[0], joint_friction_range[1])

        return asset, dof_props_asset, rigid_shape_props_asset
    