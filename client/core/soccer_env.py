import gym
from gym import error, spaces, utils
from gym.utils import seeding
import numpy as np

import grpc
import logging
logger = logging.getLogger(__name__)

import soccer3d_pb2
import soccer3d_pb2_grpc

class SoccerEnv(gym.Env, utils.EzPickle):
    metadata = {'render.modes': ['human']}

    def __init__(self, id=0, eval=False):
        # Start connection with server
        if eval:
            id += 10
        self.channel = grpc.insecure_channel('localhost:' + str(5000 + id))
        self.stub = soccer3d_pb2_grpc.DDPGTrainerStub(self.channel)

        setup = self.stub.SetupEnvironment(soccer3d_pb2.SetupEnvRequest())
        logger.info(setup.num_state_dim, setup.num_action_dim, setup.action_bound)

        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=setup.num_state_dim)
        self.action_space = spaces.Box(low=-np.array(setup.action_bound),
                                       high=np.array(setup.action_bound))


        self._spec = gym.envs.registration.EnvSpec("SoccerEnv-v0", timestep_limit=50)

    def _step(self, action):
        response = self.stub.Simulate(soccer3d_pb2.SimulationRequest(
                    action=soccer3d_pb2.Action(action=action)
        ))
        # return (response.state.observation, response.reward, response.done, {})
        return (np.array(response.state.observation), response.reward, response.done, {})

    def _reset(self):
        response = self.stub.StartEpisode(soccer3d_pb2.EpisodeRequest())
        return np.array(response.state.observation)

    def _render(self, mode='human', close=False):
        return

    def _close(self):
        response = self.stub.CloseEnvironment(soccer3d_pb2.CloseRequest())
        return

    def _seed(self, seed=None):
        return 