import glob
import logging
import os
import random
import time

from gym import spaces
from gym.spaces import Box
from sumolib import checkBinary

from aienvs.Environment import Env
from aienvs.Sumo.SumoHelper import SumoHelper
from aienvs.Sumo.TrafficLightPhases import TrafficLightPhases
from aienvs.Sumo.state_representation import *


class SumoGymAdapter(Env):
    """
    An adapter that makes Sumo behave as a proper Gym environment.
    At top level, the actionspace and percepts are in a Dict with the
    trafficPHASES as keys.

    @param maxConnectRetries the max number of retries to connect.
        A retry is needed if the randomly chosen port
        to connect to SUMO is already in use.
    """
    _DEFAULT_PARAMETERS = {'gui':True,  # gui or not
                'scenarios_path': os.path.join(os.path.dirname(__file__), "../../scenarios/Sumo/"),
                'scene':'four_grid',  # subdirectory in the aienvs/scenarios/Sumo directory where
                'tlphasesfile': None,  # Use None to read phases from net file, otherwise only relative name
                'box_bottom_corner':(0, 0),  # bottom left corner of the observable frame
                'box_top_corner':(10, 10),  # top right corner of the observable frame
                'resolutionInPixelsPerMeterX': 1,  # for the observable frame
                'resolutionInPixelsPerMeterY': 1,  # for the observable frame
                'y_t': 6,  # yellow time
                'generate_conf': True,  # for automatic route/config generation
                'simulation_start_time': '0', # The start time of the sumo simulation in seconds
                'reward_range': [100],
                'route_generation_method': 'undefined', # One of ['legacy', 'randomTrips.py', 'activitygen']

                # Options for 'route_generation_method' 'activitygen'
                'activitygen_options': [], # e.g. ["--end", endtime]
                'stat_file': None, # stat file used. Leave none for automatic search.

                # Options for 'route_generation_method' 'randomTrips.py'
                'trips_generate_options': [], # sumo/tools/randomTrips.py additional options. -n, -o, --validate already handled!

                # Custom route and trip generation if route_generation_method is set to 'legacy'
                'car_pr': 0.5,  # for automatic route/config generation probability that a car appears
                'car_tm': 2,  #  for automatic route/config generation when the first car appears?
                'route_starts' : [],  #  for automatic route/config generation, ask Rolf
                'route_min_segments' : 0,  #  for automatic route/config generation, ask Rolf
                'route_max_segments' : 0,  #  for automatic route/config generation, ask Rolf
                'route_ends' : [],  #  for automatic route/config generation, ask Rolf

                'seed': None, # Used to seed sumo, and to generate the traffic by all generation methods.

                'libsumo' : False,  # whether libsumo is used instead of traci
                'waiting_penalty' : 1,  # penalty for waiting
                'new_reward': False,  # some other type of reward ask Miguel
                'lightPositions' : {},  # specify traffic light positions
                'scaling_factor' : 1.0,  # for rescaling the reward? ask Miguel
                'maxConnectRetries':50,  # maximum reattempts to connect by Traci
                'seed': None,
                'reward_function': "default", #options include default, eval and elise
                'maxConnectRetries': 50  # maximum reattempts to connect by Traci
                }

    def __init__(self, parameters:dict={}, init_state=True):
        """
        @param path where results go, like "Experiment ID"
        @param parameters the configuration parameters.
        gui: whether we show a GUI.
        scenario: the path to the scenario to use
        """
        logging.debug(parameters)
        self._parameters = copy.deepcopy(self._DEFAULT_PARAMETERS)
        self._parameters.update(parameters)

        self._scenario_path = os.path.join(self._parameters['scenarios_path'], self._parameters['scene'])

        if self._parameters['tlphasesfile'] is None:
            tlPhasesFile = self._parameters['tlphasesfile'] = self.get_net_file()
        else:
            tlPhasesFile = os.path.join(self._parameters['scenarios_path'], self._parameters['scene'], self._parameters['tlphasesfile'])

        self._tlphases = TrafficLightPhases(tlPhasesFile)

        self.ldm = ldm(using_libsumo=self._parameters['libsumo'])
        self._takenActions = {}
        self._yellowTimer = {}
        self._chosen_action = None
        self.seed(parameters['seed'])  # in case no seed is given
        self._action_space = self._getActionSpace()

        # TODO: Wouter: make state configurable ("state factory")
        if init_state:
            self._state = LdmMatrixState(self.ldm, [self._parameters['box_bottom_corner'], self._parameters['box_top_corner']], self._parameters["reward_range"], "byCorners")
        else:
            self._state = None

        # Computed when needed instead of in __init__:
        self._observation_space = None

    def update_parameters(self, updated_params: dict):
        """
        Updates the parameters. Please note that some of the
        parameter changes may only be propagated after resetting
        the environment.
        """
        self._parameters.update(updated_params)

    def set_stat_file(self, stat_file: str):
        self.update_parameters({"stat_file": stat_file})

    def _compute_observation_space(self):
        self._startSUMO(gui=False)
        _s = self._observe()
        self.frame_height = _s.shape[0]
        self.frame_width = _s.shape[1]
        return Box(low=0, high=1.0, shape=(self.frame_height, self.frame_width), dtype=np.float32)

    def step(self, actions:dict):
        self._set_lights(actions)
        self.ldm.step()
        obs = self._observe()
        done = self.ldm.isSimulationFinished()
        global_reward_list = self._computeGlobalReward(self._parameters['reward_function'])
        if len(self._parameters['reward_range']) == 1:
            return obs, global_reward_list[self._parameters['reward_range'][0]], done, []
        else:
            return obs, global_reward_list, done, []

        # as in openai gym, last one is the info list
        # return obs, global_reward, done, []



    def reset(self):
        try:
            logging.debug("LDM closed by resetting")
            self.ldm.close()
        except:
            logging.debug("No LDM to close. Perhaps it's the first instance of training")

        logging.debug("Starting SUMO environment...")
        self._startSUMO()

        return self._observe()

        # TODO: change the defaults to something sensible
    def render(self, delay=0.0):
        import colorama
        colorama.init()

        def move_cursor(x, y):
            print ("\x1b[{};{}H".format(y + 1, x + 1))

        def clear():
            print ("\x1b[2J")

        clear()
        move_cursor(100, 100)
        import numpy as np
        np.set_printoptions(linewidth=100)
        print(self._observe())
        time.sleep(delay)

    def seed(self, seed):
        self._seed = seed

    def close(self):
        self.__del__()

    @property
    def observation_space(self):
        # # this is the previous method, which does not take resolution into consideration
        # size = self._state.size()
        # return Box(low=0, high=np.inf, shape=(size[0], size[1]), dtype=np.int32)

        if self._observation_space is None:
            self._observation_space = self._compute_observation_space()

        return self._observation_space

    @property
    def action_space(self):
        return self._action_space

    ########## Private functions ##########################
    def __del__(self):
        logging.debug("LDM closed by destructor")
        if 'ldm' in locals():
            self.ldm.close()

    def _startSUMO(self, gui=None):
        """
        Start the connection with SUMO as a subprocess and initialize
        the traci port, generate route file.
        """
        val = 'sumo'
        if gui is True:
            val = 'sumo-gui'
        elif gui is None:
            val = 'sumo-gui' if self._parameters['gui'] else 'sumo'
        
        maxRetries = self._parameters['maxConnectRetries']
        sumo_binary = checkBinary(val)

        # Try repeatedly to connect
        while True:
            try:
                # this cannot be seeded
                self._port = random.SystemRandom().choice(list(range(10000, 20000)))
                self._sumo_helper = SumoHelper(self._parameters, self._port, self._seed)
                conf_file = self._sumo_helper.sumocfg_file
                logging.debug("Configuration: " + str(conf_file))
                sumoCmd = [sumo_binary, "-c", conf_file, "-W", "-v", "false"] # shut up SUMO
                if self._seed is not None:
                    sumoCmd += ["--seed", str(self._seed)]
                self.ldm.start(sumoCmd, self._port)
            except Exception as e:
                if str(e) == "connection closed by SUMO" and maxRetries > 0:
                    maxRetries = maxRetries - 1
                    continue
                else:
                    raise
            else:
                break

        self.ldm.init(waitingPenalty=self._parameters['waiting_penalty'], new_reward=self._parameters['new_reward'])  # ignore reward for now
        self.ldm.setResolutionInPixelsPerMeter(self._parameters['resolutionInPixelsPerMeterX'], self._parameters['resolutionInPixelsPerMeterY'])
        self.ldm.setPositionOfTrafficLights(self._parameters['lightPositions'])

        if list(self.ldm.getTrafficLights()) != self._tlphases.getIntersectionIds():
            raise Exception("environment traffic lights do not match those in the tlphasesfile "
                    +self._parameters['tlphasesfile'] + str(self.ldm.getTrafficLights())
                    +str(self._tlphases.getIntersectionIds()))

    def _intToPhaseString(self, intersectionId:str, lightPhaseId: int):
        """
        @param intersectionid the intersection(light) id
        @param lightvalue the PHASES value
        @return the intersection PHASES string eg 'rrGr' or 'GGrG'
        """
        logging.debug("lightPhaseId" + str(lightPhaseId))
        return self._tlphases.getPhase(intersectionId, lightPhaseId)

    def _observe(self):
        """
        Fetches the Sumo state and converts in a proper gym observation.
        The keys of the dict are the intersection IDs (roughly, the trafficLights)
        The values are the state of the TLs
        """
        return self._state.update_state()

    def _computeGlobalReward(self, function):
        """
        Computes the global reward
        """

        rewards: dict = self._state.update_reward(function)

        for k in rewards.keys():
            rewards[k] = rewards[k] / self._parameters['scaling_factor']

        return rewards

    def _getActionSpace(self):
        """
        @returns the actionspace: a dict containing <id,phases> where
        id is the intersection id and value is
         all possible actions for each id as specified in tlphases
        """
        return spaces.Dict({inters:spaces.Discrete(self._tlphases.getNrPhases(inters)) \
                            for inters in self._tlphases.getIntersectionIds()})

    def _set_lights(self, actions:spaces.Dict):
        """
        Take the specified actions in the environment
        @param actions a list of
        """
        for intersectionId in actions.keys():
            action = self._intToPhaseString(intersectionId, actions.get(intersectionId))
            # Retrieve the action that was taken the previous step
            try:
                prev_action = self._takenActions[intersectionId]
            except KeyError:
                # If KeyError, this is the first time any action was taken for this intersection
                prev_action = action
                self._takenActions.update({intersectionId:action})
                self._yellowTimer.update({intersectionId:0})

            # Check if the given action is different from the previous action
            if prev_action != action:
                # Either the this is a true switch or coming grom yellow
                action, self._yellowTimer[intersectionId] = self._correct_action(prev_action, action, self._yellowTimer[intersectionId])

            # Set traffic lights
            self.ldm.setRedYellowGreenState(intersectionId, action)
            self._takenActions[intersectionId] = action

    def _correct_action(self, prev_action, action, timer):

        """
        Check what we are going to do with the given action based on the
        previous action.
        """
        # Check if the agent was in a yellow state the previous step
        if 'y' in prev_action:
            # Check if this agent is in the middle of its yellow state
            if timer > 0:
                new_action = prev_action
                timer -= 1
            # Otherwise we can get out of the yellow state
            else:
                new_action = self._chosen_action
                if not isinstance(new_action, str):
                    raise Exception("chosen action is illegal")
        # We are switching from green to red, initialize the yellow state
        else:
            self._chosen_action = action
            if self._parameters['y_t'] > 0:
                new_action = prev_action.replace('G', 'y')
                timer = self._parameters['y_t'] - 1
            else:
                new_action = action
                timer = 0

        return new_action, timer

    # Returns full path
    def get_net_file(self):
        net_files = glob.glob(self._scenario_path + '/*.net.xml')

        assert len(net_files) == 1, f"Expected exactly one netfile, but netfiles: {net_files}"

        return net_files[0]