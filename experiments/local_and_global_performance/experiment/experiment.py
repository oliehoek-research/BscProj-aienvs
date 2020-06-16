"""Execute the simulation experiment

This module can be used to perform the 32 simulations, compute the nash
equilibria, and compute the social optima
"""

from avg_waiting_time import awt_at_intersections
import os
import subprocess
from typing import List


def run_simulations():
    """Run the 32 SUMO simulations as described in the simulation section

    Args:
        None

    Returns:
        None
    """
    os.mkdir("output")
    for i in range(32):
        cmd = r"sumo --no-warnings --configuration-file simulations/simulation_{}/osm.sumocfg".format(i)
        subprocess.call(cmd, shell=True)


def compute_average_waiting_times() -> List[List[float]]:
    """Compute the average waiting time that occured at each intersection
    during each simulation

    Args:
        None

    Returns:
        List[List[float]]: List containing the average waiting times that occured
        for each strategy profile
    """
    results: List[List[float]] = []

    for i in range(32):
        data_file = r"output/simulation_{}_lane_data.xml".format(i)
        net_file = r"simulations/simulation_{}/osm.net.xml".format(i)
        simulation_result = awt_at_intersections(data_file, net_file, 0, 150000)
        _, awts = zip(*simulation_result)
        results.append(awts)

    return results


def compute_nash_equilibria(awts: List[List[float]]) -> List[int]:
    """Compute the nash equilibria

    Args:
        awts (List[List[float]]): List containing the average waiting times that
            occured for each strategy profile

    Returns:
        List[int]: List containing the strategy profiles that are a nash equilibrium
    """
    nash_equilibria = []

    for a in range(len(awts)):
        is_nash_equilibrium = True
        for b in range(len(awts[0])):
            for c in range(len(awts)):
                if awts[a][b] > awts[c][b]:
                    is_nash_equilibrium = False
        if is_nash_equilibrium:
            nash_equilibria.append(a)

    return nash_equilibria


def compute_social_optima(awts: List[List[float]]) -> List[int]:
    """Compute the social optima

    Args:
        awts (List[List[float]]): List containing the average waiting times that
            occured for each strategy profile

    Returns:
        List[int]: List containing the strategy profiles that are a social optimum
    """
    social_optima = []

    total_awts = [sum(awts[i]) for i in range(len(awts))]
    min_awt = min(total_awts)

    for i in range(len(total_awts)):
        if total_awts[i] == min_awt:
            social_optima.append(i)

    return social_optima


def main():
    """Runs the 32 simulations, prints the nash equilibria, and
    prints the the social optima

    Args:
        None

    Returns:
        None
    """
    print("\n\n#######################################")
    print("######### Starting experiment #########")
    print("#######################################\n\n", flush=True)
    run_simulations()

    print("\n\n#######################################")
    print("### Computing average waiting times ###")
    print("#######################################\n\n", flush=True)
    awts = compute_average_waiting_times()

    for idx, sp_awt in enumerate(awts):
        print("Experiment {}:".format(idx), sp_awt)

    print("\n\n#######################################")
    print("###### Computing Nash equilibria ######")
    print("#######################################\n\n", flush=True)
    nash_equilibria: List[int] = compute_nash_equilibria(awts)

    if len(nash_equilibria) == 0:
        print("There are no Nash equilibria")
    else:
        for i in nash_equilibria:
            print("The strategy profile from simulation {} is a Nash equilibrium".format(i))

    print("\n\n#######################################")
    print("####### Computing social optima #######")
    print("#######################################\n\n", flush=True)
    social_optima: List[int] = compute_social_optima(awts)

    for i in social_optima:
        print("The strategy profile from simulation {} results in a social optimum".format(i))

    print("\n")


if __name__ == "__main__":
    main()
