from typing import Any, override

import numpy as np
from gymnasium import utils
from nao_interface.poses import PENALIZED_POSE
from numpy.typing import NDArray
from rewards import (
    ControlAmplitudePenalty,
    ExternalImpactForcesPenalty,
    HeadHeightReward,
    RewardComposer,
    TorqueChangeRatePenalty,
)
from rewards.base import RewardContext

from .nao_base_env import NaoBaseEnv


class NaoStandup(NaoBaseEnv, utils.EzPickle):
    def __init__(self, *, throw_tomatoes: bool = False, **kwargs: Any) -> None:
        super().__init__(throw_tomatoes=throw_tomatoes, **kwargs)
        self.reward = (
            RewardComposer()
            .add(1.0, HeadHeightReward())
            .add(-0.1, ControlAmplitudePenalty())
            .add(-0.5e-6, ExternalImpactForcesPenalty())
            .add(-0.01, TorqueChangeRatePenalty(self.model.nu, self.dt))
        )
        utils.EzPickle.__init__(self, **kwargs)

    @override
    def step(self, action: NDArray[np.floating]) -> tuple:
        self.do_simulation(action, self.frame_skip)

        distinct_rewards = self.reward.rewards(RewardContext(self.nao, action))
        reward = sum(distinct_rewards.values())

        if self.render_mode == "human":
            self.render()

        return (self._get_obs(), reward, False, False, distinct_rewards)

    @override
    def reset_model(self) -> NDArray[np.floating]:
        self.set_state(
            self.init_qpos,
            self.init_qvel,
        )
        self.nao.reset(PENALIZED_POSE)
        self.nao.set_transform(
            np.array([-0.13252355, -0.0909888, 0.05897925]),
            np.array([0.69360432, 0.13973604, -0.692682, 0.13992331]),
        )

        return self._get_obs()
