import numpy as np
from gym import utils
from gym.envs.mujoco import mujoco_env

from .example_models import twod_corridor

class TwoDCorridorEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        model = twod_corridor()
        with model.asfile() as f:
            mujoco_env.MujocoEnv.__init__(self, model_path=f.name, frame_skip=5)
        utils.EzPickle.__init__(self)

    def _step(self, action):
        self.do_simulation(action, self.frame_skip)
        ob = self._get_obs()
        reward = 0
        done = False
        return ob, reward, done, {}

    def _get_obs(self):
        return np.concatenate([self.model.data.qpos]).ravel()

    def reset_model(self):
        qpos = self.init_qpos + self.np_random.uniform(low=-.1, high=.1, size=self.model.nq)
        qvel = self.init_qvel + self.np_random.randn(self.model.nv) * .1
        self.set_state(qpos, qvel)
        return self._get_obs()

    def viewer_setup(self):
        pass
