from cbs_basic import CBSSolver
from a_star_class import compute_heuristics

def main():
    # Define the map (0 for empty, 1 for obstacle)
    my_map = [
        [0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0]
    ]

    # Define start and goal positions for three agents
    starts = [(0, 0), (0, 4), (4, 4)]
    goals = [(4, 4), (4, 0), (0, 0)]

    # Compute heuristics for each agent
    heuristics = []
    for goal in goals:
        heuristics.append(compute_heuristics(my_map, goal))

    # Create a CBS solver instance
    cbs = CBSSolver(my_map, starts, goals)

    # Find solution
    solution = cbs.find_solution(disjoint=False)

    if solution is not None:
        paths, num_generated, num_expanded = solution
        print("\nFound a solution!")
        print("Paths:")
        for i, path in enumerate(paths):
            print(f"Agent {i}: {path}")
        print(f"Nodes generated: {num_generated}")
        print(f"Nodes expanded: {num_expanded}")
    else:
        print("\nNo solution found!")

if __name__ == '__main__':
    main()