"""Calculate average waiting time

This module can be used to calculate the average waiting time that
cars spent at the intersections during an experiment.
"""

import sys
import getopt
import xml.etree.ElementTree as ET
from typing import List, Union, Tuple


class Intersection:
    """A class used to represent an Intersection

    Args:
        name (str): The name of the intersection
        incoming_lanes (List[str]): List of strings representing the names of the incoming lanes
        waiting_time (float): The total waiting time spent at the intersection
            (default is 0.0)
        cars (int): The number of cars that used the intersection
            (default is 0)

    Attributes:
        name (str): The name of the intersection
        _inc_lanes (List[str]): List of strings representing the names of the incoming lanes
        _wait_time (float): The total waiting time spent at the intersection
        _cars (int): The number of cars that used the intersection
    """
    def __init__(self, name: str, incoming_lanes: List[str], waiting_time: float = 0.0, cars: int = 0):
        self.name: str = name
        self._inc_lanes: List[str] = incoming_lanes
        self._wait_time: float = waiting_time
        self._cars: int = cars

    def add_waiting_time(self, time: float) -> None:
        """Add additional waiting time to the total waiting time spent at the intersection

        Args:
            time (float): The time to add to the total waiting time

        Returns:
            None
        """
        self._wait_time += time

    def add_cars(self, cars: int) -> None:
        """Add additional cars to the total cars that used the intersection

        Args:
            cars (int): The cars to add to the total cars

        Returns:
            None
        """
        self._cars += cars

    def average_waiting_time(self) -> float:
        """Compute average waiting time cars spent at the intersection

        Args:
            None

        Returns:
            float: The average waiting time cars spent at the intersection
        """
        return self._wait_time / self._cars if self._cars else 0

    def contains_incoming_lane(self, lane: str) -> bool:
        """Check if a lane is an incoming lane of the intersection

        Args:
            lane (str): The name of the lane to check

        Returns:
            bool: True if the lane is an incoming lane, False otherwise
        """
        return lane in self._inc_lanes


def parse_command_line_arguments(argv: List[str]) -> Tuple[str, str, float, float]:
    """Extracts, and checks the exists of, arguments needed for calculating the average waiting time
    at intersections using file descriptions

    Args:
        argv (List[str]): List containing the command-line arguments
            -d (--datafile): The name of the datafile
            -n (--netfile): The name of the netfile
            -s (--starttime): The minimum start time of the information to check about the lanes
            -e (--endtime): The maximum end time of the information to check about the lanes
            -h (--help): Print information about the command line arguments

    Returns:
        Tuple[str,str,float,flaot]: Tuple containig the names of the data and net files, and the start and end time
    """
    try:
        opts, args = getopt.getopt(argv, "d:n:s:e:h", ["datafile=", "netfile=", "starttime=", "endtime=", "help="])
    except getopt.GetoptError:
        print('statistics.py -d <datafile> -n <netfile> -s <starttime> -e <endtime>')
        sys.exit(2)
    for opt, arg in opts:
        if opt in ("-h", "--help"):
            print('statistics.py -d <datafile> -n <netfile> -s <starttime> -e <endtime>')
            print('-d (--datafile): The name of the datafile')
            print('-n (--netfile): The name of the netfile')
            print('-s (--starttime): The minimum start time of the information to check about the lanes')
            print('-e (--endtime): The maximum end time of the information to check about the lanes')
            print('-h (--help): Print information about the command line arguments')
            sys.exit()
        elif opt in ("-d", "--datafile"):
            data_file: str = arg
        elif opt in ("-n", "--netfile"):
            net_file: str = arg
        elif opt in ("-s", "--starttime"):
            start_time: float = float(arg)
        elif opt in ("-e", "--endtime"):
            end_time: float = float(arg)

    if 'data_file' not in locals():
        sys.exit('Data file unspecified: use -d or --datafile')
    if 'net_file' not in locals():
        sys.exit('Net file unspecified: use -n or --netfile')
    if end_time < start_time:
        sys.exit('The end time should be bigger or equal to the start time')

    return data_file, net_file, start_time, end_time


def awt_at_intersections(data_file: str, net_file: str, start_time: float, end_time: float) -> List[Tuple[str, float]]:
    """Computes the average waiting time for every specified intersection in the net file

    Args:
        data_file: The name of the datafile
        net_file: The name of the netfile
        start_time: The minimum start time of the information to check about the lanes
        end_time:The maximum end time of the information to check about the lanes

    Returns:
        List[Tuple[str, float]]: List of tuples that contain the name of, and average waiting time
        at, the intersections
    """
    # create intersections from net file content
    intersections: List[Intersection] = []
    net_file_tree: ET.ElementTree = ET.parse(net_file)
    net_file_content: ET.Element = net_file_tree.getroot()
    for junction in net_file_content.findall('junction'):
        if junction.attrib['type'] == "traffic_light":
            name = junction.attrib['id']
            incoming_lanes: List[str] = junction.attrib['incLanes'].split()
            intersections.append(Intersection(name, incoming_lanes))

    # load data file content
    data_file_tree: ET.ElementTree = ET.parse(data_file)
    data_file_content: ET.Element = data_file_tree.getroot()

    # look at the information of every lane that is known between the begin and end time
    for interval in data_file_content.findall('interval'):
        if float(interval.attrib['begin']) >= start_time and float(interval.attrib['end']) <= end_time:
            for edge in interval:
                for lane in edge:
                    lane_name: str = lane.attrib['id']
                    # add the information of the lane to the corresponding intersection if said intersection is defined
                    intersection: Union[Intersection, None] = next(
                        (i for i in intersections if i.contains_incoming_lane(lane_name)),
                        None
                    )
                    if intersection is not None:
                        intersection.add_waiting_time(float(lane.attrib['waitingTime']))
                        intersection.add_cars(int(lane.attrib['left']))

    result: List[Tuple[str, float]] = []
    for intersection in intersections:
        result.append((intersection.name, round(intersection.average_waiting_time(), 2)))
    return result


def main(argv: List[str]):
    """Prints the average waiting time for every intersection

    Args:
        argv (List[str]): List containing the command-line arguments

    Returns:
        None
    """
    data_file, net_file, start_time, end_time = parse_command_line_arguments(argv)
    intersections = awt_at_intersections(data_file, net_file, start_time, end_time)
    for name, avg_waiting_time in intersections:
        print(name, avg_waiting_time)


if __name__ == "__main__":
    main(sys.argv[1:])
