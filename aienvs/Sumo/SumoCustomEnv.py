import copy
import os

from gym.vector.utils import spaces

from aienvs.Sumo.NetFileUtil import NetFileUtil
from aienvs.Sumo.state_representation import LdmMatrixState
from .SumoGymAdapter import SumoGymAdapter


class SumoCustomEnv(SumoGymAdapter):
    __DEFAULT_PARAMETERS = {
        'scenarios_path': os.path.join(os.path.dirname(__file__), "../../scenarios/Sumo/"),
        'scene': 'rp-scenario-v2',
        'resolutionInPixelsPerMeterX': 0.25,
        'resolutionInPixelsPerMeterY': 0.25,
        'box_width': 100,
        'box_height': 100,
        'route_generation_method': 'activitygen',  # One of ['legacy', 'randomTrips.py', 'activitygen']

    }

    def __init__(self, parameters: dict = None, junction_id: str = "Waring_Woodhull"):
        """
        Class for creating a Sumo environment based on a custom (imported from OSM) scenario.
        :param parameters: Dict with parameters.
        :param junction_id: The junction that should be controllable by an agent.
        """
        _parameters = copy.deepcopy(self._DEFAULT_PARAMETERS)  # load default parameters of SUMOGymAdaptor
        _parameters.update(self.__DEFAULT_PARAMETERS)  # load default parameters of GridSumoEnv
        _parameters.update(parameters)  # load parameters given by the user

        # Set tlsphasesfile to None, to use the net file
        _parameters['tlphasesfile'] = None

        super().__init__(parameters=_parameters)

        net_file = self.get_net_file()

        nfu = NetFileUtil(net_file)

        junction = nfu.find_junction(junction_id)

        self.controlled_agent = nfu.find_tlid_associated_with_junction(junction)

        self.set_observation_box_around_point(float(junction['x']), float(junction['y']),
                                              self._parameters['box_width'], self._parameters['box_height'])

        self._state = LdmMatrixState(self.ldm,
                                     [self._parameters['box_bottom_corner'], self._parameters['box_top_corner']],
                                     "byCorners")

    def set_observation_box_around_point(self, point_x: float, point_y: float, width: float, height: float) -> None:
        """
        :param width: Width of the ldm observation/reward box
        :param height: Height of the ldm observation/reward box
        :param point_x: Center of box x-coordinate
        :param point_y: Center of box y-coordinate
        """
        bottom_left = (point_x - 0.5 * width, point_y - 0.5 * height)
        top_right = (point_x + 0.5 * width, point_y + 0.5 * height)

        self._parameters['box_bottom_corner'] = bottom_left
        self._parameters['box_top_corner'] = top_right

    @SumoGymAdapter.action_space.getter
    def action_space(self) -> spaces.Dict:
        """
        Action space filtered to only return controlled intersection.
        :return: Dict: {Controlled intersection id, actions}
        """
        return spaces.Dict({self.controlled_agent: self._action_space[self.controlled_agent]})
