from __future__ import print_function
from solver import *
from input_parser import *
from constants import *
from brute_force import *
import timeit


def find_routing(coords):
    start = timeit.default_timer()

    # Create Routing Model
    routing = pywrapcp.RoutingModel(len(coords), 1, 0)

    metric = algorithm_ID
    # Define weight of each edge
    if algorithm_ID > 2:
        metric = 0
    distance_callback = create_distance_callback(metric, coords)
    routing.SetArcCostEvaluatorOfAllVehicles(distance_callback)
    add_distance_dimension(routing, distance_callback)

    # Setting first solution heuristic (cheapest addition).
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()

    if algorithm_ID == 3:
        search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES
    elif algorithm_ID == 4:
        search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.LOCAL_CHEAPEST_ARC
    elif algorithm_ID == 5:
        search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.SAVINGS
    else:
        search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    if assignment:
        solution, overallDistance, points = get_solution(coords, routing, assignment)
        t = timeit.default_timer() - start
        print_solution_on_console(solution, overallDistance, t)
        save_to_output_file(input_file, points, overallDistance, t)


if __name__ == '__main__':
    input_file, algorithm_ID = parse_input()
    with open(str(input_file)) as f:
        locations = [tuple(map(float, i.split(','))) for i in f]
    if algorithm_ID != BRUTE_FORCE:
        find_routing(locations)
    else:
        brute_force(locations)
