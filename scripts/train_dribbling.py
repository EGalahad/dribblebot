def train_go1(headless=True):

    import isaacgym
    assert isaacgym
    import torch

    from dribblebot.envs.base.legged_robot_config import Cfg
    from dribblebot.envs.go1.go1_config import config_go1
    from dribblebot.envs.cyberdog2.cyberdog2_config import config_cyberdog2
    from dribblebot.envs.go1.velocity_tracking import VelocityTrackingEasyEnv

    from dribblebot_learn.ppo_cse import Runner
    from dribblebot.envs.wrappers.history_wrapper import HistoryWrapper
    from dribblebot_learn.ppo_cse.actor_critic import AC_Args
    from dribblebot_learn.ppo_cse.ppo import PPO_Args
    from dribblebot_learn.ppo_cse import RunnerArgs

    config_cyberdog2(Cfg)
    RunnerArgs.resume = False
    RunnerArgs.resume_path = ""
    RunnerArgs.resume_checkpoint = 'tmp/legged_data/ac_weights_last.pt' 

    # may change number of environments
    Cfg.env.num_envs = 3000
    num_learning_iterations = 10000

    Cfg.domain_rand.model_unseen = True
    Cfg.noise.add_noise = True
    Cfg.noise.noise_level = 1
    Cfg.noise_scales.ball_pos = 0.1


    # may change number of history frames and privileged observations
    Cfg.env.num_observation_history = 15
    Cfg.env.num_privileged_obs = 6


    Cfg.sensors.privileged_sensor_names = {
                        "BodyVelocitySensor": {},
                        "ObjectVelocitySensor": {},
    }
    Cfg.sensors.privileged_sensor_args = {
                        "BodyVelocitySensor": {},
                        "ObjectVelocitySensor": {},
    }

    #? what are these parameter for ?
    Cfg.commands.num_lin_vel_bins = 30
    Cfg.commands.num_ang_vel_bins = 30
    Cfg.commands.distributional_commands = True

    Cfg.rewards.relative_use_head = True

    Cfg.curriculum_thresholds.tracking_ang_vel = 0.7
    Cfg.curriculum_thresholds.tracking_lin_vel = 0.8
    Cfg.curriculum_thresholds.tracking_contacts_shaped_vel = 0.90
    Cfg.curriculum_thresholds.tracking_contacts_shaped_force = 0.90
    Cfg.curriculum_thresholds.dribbling_ball_vel = 0.8

    # domain randomization ranges
    Cfg.domain_rand.rand_interval_s = 6

    Cfg.domain_rand.lag_timesteps = 6
    Cfg.domain_rand.randomize_lag_timesteps = True

    # randomize the rigid shape props of robot joints
    Cfg.domain_rand.randomize_rigids_after_start = False
    Cfg.domain_rand.randomize_friction = False
    Cfg.domain_rand.randomize_restitution = True
    Cfg.domain_rand.friction_range = [0.0, 1.5]
    Cfg.domain_rand.restitution_range = [0.0, 0.4]

    Cfg.domain_rand.randomize_base_mass = True
    Cfg.domain_rand.added_mass_range = [-1.0, 3.0]
    Cfg.domain_rand.randomize_com_displacement = False
    Cfg.domain_rand.com_displacement_range = [-0.15, 0.15]

    # randomize motor control
    Cfg.domain_rand.randomize_motor_strength = True
    Cfg.domain_rand.motor_strength_range = [0.99, 1.01]
    Cfg.domain_rand.randomize_motor_offset = True
    Cfg.domain_rand.motor_offset_range = [-0.002, 0.002]
    Cfg.domain_rand.randomize_Kp_factor = True
    Cfg.domain_rand.Kp_factor_range = [0.8, 1.2]
    Cfg.domain_rand.randomize_Kd_factor = True
    Cfg.domain_rand.Kd_factor_range = [0.8, 1.2]

    # external forces
    Cfg.domain_rand.push_robots = True
    Cfg.domain_rand.max_push_vel_xy = 0.5
    
    # randomize the ball properties
    Cfg.domain_rand.randomize_ball_friction = True
    Cfg.domain_rand.randomize_ball_restitution = True
    Cfg.domain_rand.randomize_ball_drag = True
    Cfg.domain_rand.ball_friction_range = [0.5, 1]
    Cfg.domain_rand.ball_restitution_range = [0.2, 1]
    Cfg.domain_rand.drag_range = [0.1, 0.8]
    Cfg.domain_rand.ball_drag_rand_interval_s = 15.0

    # randomize the ground properties
    Cfg.domain_rand.randomize_ground_friction = True
    Cfg.domain_rand.randomize_ground_restitution = True
    Cfg.domain_rand.ground_friction_range = [0.7, 4.0]
    Cfg.domain_rand.ground_restitution_range = [0.6, 1.0]
    Cfg.domain_rand.tile_roughness_range = [0.0, 0.0]

    # not used
    Cfg.domain_rand.randomize_gravity = False
    Cfg.domain_rand.gravity_range = [-1.0, 1.0]
    Cfg.domain_rand.gravity_rand_interval_s = 8.0
    Cfg.domain_rand.gravity_impulse_duration = 0.99

    Cfg.commands.exclusive_phase_offset = False
    Cfg.commands.pacing_offset = False
    Cfg.commands.balance_gait_distribution = False
    Cfg.commands.binary_phases = False
    Cfg.commands.gaitwise_curricula = False


    # terrain configuration
    Cfg.terrain.border_size = 0.0
    Cfg.terrain.mesh_type = "boxes_tm"
    Cfg.terrain.num_cols = 20
    Cfg.terrain.num_rows = 20
    Cfg.terrain.terrain_length = 5.0
    Cfg.terrain.terrain_width = 5.0
    Cfg.terrain.num_border_boxes = 5.0
    Cfg.terrain.x_init_range = 0.2
    Cfg.terrain.y_init_range = 0.2
    Cfg.terrain.teleport_thresh = 0.3
    Cfg.terrain.teleport_robots = False
    Cfg.terrain.center_robots = False
    Cfg.terrain.center_span = 3
    Cfg.terrain.horizontal_scale = 0.05
    Cfg.terrain.terrain_proportions = [1.0, 0.0, 0.0, 0.0, 0.0]
    Cfg.terrain.curriculum = False
    Cfg.terrain.difficulty_scale = 1.0
    Cfg.terrain.max_step_height = 0.26
    Cfg.terrain.min_step_run = 0.25
    Cfg.terrain.max_step_run = 0.4
    Cfg.terrain.max_init_terrain_level = 1

    # I do not understand the different between this block and the next
    Cfg.commands.lin_vel_x = [-1.5, 1.5]
    Cfg.commands.lin_vel_y = [-1.5, 1.5]
    Cfg.commands.ang_vel_yaw = [-0.0, 0.0]
    Cfg.commands.body_height_cmd = [-0.05, 0.05]
    Cfg.commands.gait_frequency_cmd_range = [3.0, 3.0]
    Cfg.commands.gait_phase_cmd_range = [0.5, 0.5]
    Cfg.commands.gait_offset_cmd_range = [0.0, 0.0]
    Cfg.commands.gait_bound_cmd_range = [0.0, 0.0]
    Cfg.commands.gait_duration_cmd_range = [0.5, 0.5]
    Cfg.commands.footswing_height_range = [0.09, 0.09]
    Cfg.commands.body_pitch_range = [-0.0, 0.0]
    Cfg.commands.body_roll_range = [-0.0, 0.0]
    Cfg.commands.stance_width_range = [0.0, 0.1]
    Cfg.commands.stance_length_range = [0.0, 0.1]

    Cfg.commands.limit_vel_x = [-1.5, 1.5]
    Cfg.commands.limit_vel_y = [-1.5, 1.5]
    Cfg.commands.limit_vel_yaw = [-0.0, 0.0]
    Cfg.commands.limit_body_height = [-0.05, 0.05]
    Cfg.commands.limit_gait_frequency = [3.0, 3.0]
    Cfg.commands.limit_gait_phase = [0.5, 0.5]
    Cfg.commands.limit_gait_offset = [0.0, 0.0]
    Cfg.commands.limit_gait_bound = [0.0, 0.0]
    Cfg.commands.limit_gait_duration = [0.5, 0.5]
    Cfg.commands.limit_footswing_height = [0.09, 0.09]
    Cfg.commands.limit_body_pitch = [-0.0, 0.0]
    Cfg.commands.limit_body_roll = [-0.0, 0.0]
    Cfg.commands.limit_stance_width = [0.0, 0.1]
    Cfg.commands.limit_stance_length = [0.0, 0.1]

    # I do not understand the meaning of this block
    Cfg.commands.num_bins_vel_x = 1
    Cfg.commands.num_bins_vel_y = 1
    Cfg.commands.num_bins_vel_yaw = 1
    Cfg.commands.num_bins_body_height = 1
    Cfg.commands.num_bins_gait_frequency = 1
    Cfg.commands.num_bins_gait_phase = 1
    Cfg.commands.num_bins_gait_offset = 1
    Cfg.commands.num_bins_gait_bound = 1
    Cfg.commands.num_bins_gait_duration = 1
    Cfg.commands.num_bins_footswing_height = 1
    Cfg.commands.num_bins_body_roll = 1
    Cfg.commands.num_bins_body_pitch = 1
    Cfg.commands.num_bins_stance_width = 1

    # normalization for the observations
    Cfg.normalization.friction_range = Cfg.domain_rand.friction_range
    Cfg.normalization.restitution_range = Cfg.domain_rand.restitution_range
    Cfg.normalization.ground_friction_range = Cfg.domain_rand.ground_friction_range

    AC_Args.adaptation_labels = ["body_vel", "obj_vel"]
    AC_Args.adaptation_dims = [3, 3]
    AC_Args.adaptation_weights = []

    RunnerArgs.save_video_interval = 500

    import wandb
    wandb.init(
      # set the wandb project where this run will be logged
      project="dribbling",

      # track hyperparameters and run metadata
      config={
      "AC_Args": vars(AC_Args),
      "PPO_Args": vars(PPO_Args),
      "RunnerArgs": vars(RunnerArgs),
      "Cfg": vars(Cfg),
      }
    )

    device = 'cuda:0'
    # device = 'cpu'
    env = VelocityTrackingEasyEnv(sim_device=device, headless=headless, cfg=Cfg)

    env = HistoryWrapper(env)
    runner = Runner(env, device=device)
    runner.learn(num_learning_iterations=num_learning_iterations, init_at_random_ep_len=True, eval_freq=100)


if __name__ == '__main__':
    from pathlib import Path
    from dribblebot import MINI_GYM_ROOT_DIR

    stem = Path(__file__).stem
    
    # to see the environment rendering, set headless=False
    train_go1(headless=True)

