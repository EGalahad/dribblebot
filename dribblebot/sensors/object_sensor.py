from .sensor import Sensor
import torch

class ObjectSensor(Sensor):
    def __init__(self, env, attached_robot_asset=None):
        super().__init__(env)
        self.env = env
        self.attached_robot_asset = attached_robot_asset

    def get_observation(self, env_ids = None):
        ball_pos = self.env.object_local_pos
        # [num_envs, 3]
        if self.env.cfg.domain_rand.model_unseen:
            # if ball pos is [-0.4~0.4, -0.2~0.2], then it is not visible to the robot, we will set it at [0.2, 0]
            invisible_mask = (ball_pos[:, 0] > -0.4) & (ball_pos[:, 0] < 0.4) & (ball_pos[:, 1] > -0.2) & (ball_pos[:, 1] < 0.2)
            ball_pos[invisible_mask, 0] = 0.2
            ball_pos[invisible_mask, 1] = 0
        return ball_pos * self.env.cfg.obs_scales.ball_pos
    
    def get_noise_vec(self):
        import torch
        return torch.ones(3, device=self.env.device) * self.env.cfg.noise_scales.ball_pos * self.env.cfg.noise.noise_level * self.env.cfg.obs_scales.ball_pos
    
    def get_dim(self):
        return 3