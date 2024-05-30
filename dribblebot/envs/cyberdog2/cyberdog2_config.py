from typing import Union

from params_proto import Meta

from dribblebot.envs.base.legged_robot_config import Cfg


def config_cyberdog2(Cnfg: Union[Cfg, Meta]):
    _ = Cnfg.init_state

    _.pos = [0.0, 0.0, 0.27]  # x,y,z [m]
    _.default_joint_angles = {  # = target angles [rad] when action = 0.0
        "FL_hip_joint": 0.0,  # [rad]
        "RL_hip_joint": 0.0,  # [rad]
        "FR_hip_joint": 0.0,  # [rad]
        "RR_hip_joint": 0.0,  # [rad]
        "FL_thigh_joint": -45 / 57.3,  # [rad]
        "RL_thigh_joint": -45 / 57.3,  # [rad]
        "FR_thigh_joint": -45 / 57.3,  # [rad]
        "RR_thigh_joint": -45 / 57.3,  # [rad]
        "FL_calf_joint": 70 / 57.3,  # [rad]
        "RL_calf_joint": 70 / 57.3,  # [rad]
        "FR_calf_joint": 70 / 57.3,  # [rad]
        "RR_calf_joint": 70 / 57.3,  # [rad]
    }

    # _.default_joint_angles = {  # = target angles [rad] when action = 0.0
    #     "FL_hip_joint": 0.0,  # [rad]
    #     "RL_hip_joint": 0.0,  # [rad]
    #     "FR_hip_joint": 0.0,  # [rad]
    #     "RR_hip_joint": 0.0,  # [rad]
    #     "FL_thigh_joint": -30 / 57.3,  # [rad]
    #     "RL_thigh_joint": -45 / 57.3,  # [rad]
    #     "FR_thigh_joint": -30 / 57.3,  # [rad]
    #     "RR_thigh_joint": -45 / 57.3,  # [rad]
    #     "FL_calf_joint": 60 / 57.3,  # [rad]
    #     "RL_calf_joint": 70 / 57.3,  # [rad]
    #     "FR_calf_joint": 60 / 57.3,  # [rad]
    #     "RR_calf_joint": 70 / 57.3,  # [rad]
    # }

    # _.default_joint_angles = {  # = target angles [rad] when action = 0.0
    #     "FL_hip_joint": 0.0,  # [rad]
    #     "RL_hip_joint": 0.0,  # [rad]
    #     "FR_hip_joint": 0.0,  # [rad]
    #     "RR_hip_joint": 0.0,  # [rad]
    #     "FL_thigh_joint": -50 / 57.3,  # [rad]
    #     "RL_thigh_joint": -50 / 57.3,  # [rad]
    #     "FR_thigh_joint": -50 / 57.3,  # [rad]
    #     "RR_thigh_joint": -50 / 57.3,  # [rad]
    #     "FL_calf_joint": 80 / 57.3,  # [rad]
    #     "RL_calf_joint": 80 / 57.3,  # [rad]
    #     "FR_calf_joint": 80 / 57.3,  # [rad]
    #     "RR_calf_joint": 80 / 57.3,  # [rad]
    # }
    # _.default_joint_angles = {  # = target angles [rad] when action = 0.0
    #     "FL_hip_joint": 0.0,  # [rad]
    #     "RL_hip_joint": 0.0,  # [rad]
    #     "FR_hip_joint": 0.0,  # [rad]
    #     "RR_hip_joint": 0.0,  # [rad]
    #     "FL_thigh_joint": -45 / 57.3,  # [rad]
    #     "RL_thigh_joint": -60 / 57.3,  # [rad]
    #     "FR_thigh_joint": -45 / 57.3,  # [rad]
    #     "RR_thigh_joint": -60 / 57.3,  # [rad]
    #     "FL_calf_joint": 90 / 57.3,  # [rad]
    #     "RL_calf_joint": 90 / 57.3,  # [rad]
    #     "FR_calf_joint": 90 / 57.3,  # [rad]
    #     "RR_calf_joint": 90 / 57.3,  # [rad]
    # }

    _ = Cnfg.control
    _.control_type = 'P'
    _.stiffness = {'joint': 60.}  # [N*m/rad]
    _.damping = {'joint': 2. }  # [N*m*s/rad]
    # action scale: target angle = actionScale * action + defaultAngle
    _.action_scale = 0.25
    _.hip_scale_reduction = 1.0
    # decimation: Number of control action updates @ sim DT per policy DT
    _.decimation = 4

    _ = Cnfg.asset
    _.file = '{MINI_GYM_ROOT_DIR}/resources/robots/cyberdog2/urdf/cyberdog2_v2.urdf'
    _.foot_name = "foot"
    _.penalize_contacts_on = ["thigh", "calf"]
    _.terminate_after_contacts_on = []
    _.self_collisions = 0  # 1 to disable, 0 to enable...bitwise filter
    _.flip_visual_attachments = True
    _.fix_base_link = False

    _ = Cnfg.rewards
    _.soft_dof_pos_limit = 0.9
    _.base_height_target = 0.20
    _.use_terminal_body_height = True
    _.terminal_body_height = 0.16
    _.use_terminal_roll_pitch = True
    _.terminal_body_ori = 0.5
    _.relative_use_head = False
    _.only_positive_rewards = False
    _.only_positive_rewards_ji22_style = True

    _ = Cnfg.reward_scales
    _.tracking_lin_vel = 0.0
    _.tracking_ang_vel = 0.0
    _.feet_air_time = 0.0
    _.orientation = -5.
    _.torques = -0.0001
    _.dof_pos = -0.05
    _.dof_vel = -0.0001
    _.dof_acc = -2.5e-7
    _.collision = -5.0
    _.action_rate = -0.01
    _.tracking_contacts_shaped_force = 4.0
    _.tracking_contacts_shaped_vel = 4.0
    _.dof_pos_limits = -10.0
    _.action_smoothness_1 = -0.1
    _.action_smoothness_2 = -0.1
    _.feet_slip = -0.04
    _.base_height = -10.
    _.lin_vel_z = -0.02
    _.ang_vel_xy = -0.001
    _.dribbling_robot_ball_vel = 0.5
    _.dribbling_robot_ball_pos = 4.0
    _.dribbling_robot_ball_yaw = 4.0
    _.dribbling_ball_vel = 4.0
    _.dribbling_ball_vel_norm = 4.0
    _.dribbling_ball_vel_angle = 4.0

    _ = Cnfg.terrain
    _.mesh_type = 'trimesh'
    _.measure_heights = False
    _.terrain_noise_magnitude = 0.0
    _.teleport_robots = True
    _.border_size = 50
    _.yaw_init_range = 3.14

    _.terrain_proportions = [0, 0, 0, 0, 0, 0, 0, 0, 1.0]
    _.curriculum = False

    _ = Cnfg.env
    _.num_envs = 4000
    _.num_observations = 75
    _.num_observation_history = 15
    _.num_privileged_obs = 6
    _.episode_length_s = 40.
    _.add_balls = True

    _ = Cnfg.commands
    _.num_commands = 15
    _.heading_command = False
    _.resampling_time = 7.0
    _.command_curriculum = True
    _.num_lin_vel_bins = 30
    _.num_ang_vel_bins = 30
    _.lin_vel_x = [-0.6, 0.6]
    _.lin_vel_y = [-0.6, 0.6]
    _.ang_vel_yaw = [-1, 1]

    _ = Cnfg.domain_rand
    _.randomize_base_mass = True
    _.added_mass_range = [-1, 3]
    _.push_robots = True
    _.max_push_vel_xy = 0.5
    _.randomize_friction = True
    _.friction_range = [0.05, 4.5]
    _.randomize_restitution = True
    _.restitution_range = [0.0, 1.0]
    _.restitution = 0.5  # default terrain restitution
    _.randomize_com_displacement = True
    _.com_displacement_range = [-0.1, 0.1]
    _.randomize_motor_strength = True
    _.motor_strength_range = [0.9, 1.1]
    _.randomize_Kp_factor = False
    _.Kp_factor_range = [0.8, 1.3]
    _.randomize_Kd_factor = False
    _.Kd_factor_range = [0.5, 1.5]
    _.rand_interval_s = 6
    _.joint_damping_range = [0.01, 0.05]
    _.joint_friction_range = [0.01, 0.05]
    _.detection_decimation = 10

    _ = Cnfg.normalization
    _.clip_actions = 100

    _ = Cnfg.ball
    _.mass = 0.32
    _.radius = 0.1225
    _.ball_init_pos = [0.0, 0.0, 0.15]
    _.init_pos_range = [7.0, 7.0, 0.05]
    
    _ = Cnfg.robot
    _.name = 'cyberdog2'

    _ = Cnfg.sensors
    _.sensor_names = [
                        "ObjectSensor",
                        "OrientationSensor",
                        "RCSensor",
                        "JointPositionSensor",
                        "JointVelocitySensor",
                        "ActionSensor",
                        "LastActionSensor",
                        "ClockSensor",
                        "YawSensor",
                        "TimingSensor",
                        ]
    _.sensor_args = {
                        "ObjectSensor": {},
                        "OrientationSensor": {},
                        "RCSensor": {},
                        "JointPositionSensor": {},
                        "JointVelocitySensor": {},
                        "ActionSensor": {},
                        "LastActionSensor": {"delay": 1},
                        "ClockSensor": {},
                        "YawSensor": {},
                        "TimingSensor":{},
                        }