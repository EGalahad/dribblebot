from .sensor import Sensor

class ObjectSensor(Sensor):
    def __init__(self, env, attached_robot_asset=None, decimation=1):
        super().__init__(env)
        self.env = env
        self.attached_robot_asset = attached_robot_asset
        self.counter = -1
        self.cached_observation = None
        self.decimation = decimation
        print(f"ObjectSensor with decimation {decimation}")

    def get_observation(self, env_ids = None):
        self.counter += 1
        if self.counter % self.decimation == 0:
            self.cached_observation = self.env.object_local_pos * self.env.cfg.obs_scales.ball_pos
        return self.cached_observation

    def get_noise_vec(self):
        import torch
        return torch.ones(3, device=self.env.device) * self.env.cfg.noise_scales.ball_pos * self.env.cfg.noise.noise_level * self.env.cfg.obs_scales.ball_pos

    def get_dim(self):
        return 3