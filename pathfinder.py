import sys
import heapq
from collections import deque
from math import sqrt

class Node:
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

    def __lt__(self, other):
        return self.path_cost < other.path_cost

    def __eq__(self, other):
        return self.state == other.state

def read_map(filename):
    with open(filename, 'r') as file:
        rows, cols = map(int, file.readline().split())
        start_pos = tuple(map(int, file.readline().split()))
        end_pos = tuple(map(int, file.readline().split()))
        grid = [list(map(str.strip, file.readline().split())) for _ in range(rows)]
    return rows, cols, start_pos, end_pos, grid

def print_map(grid):
    for row in grid:
        print(' '.join(row))

def is_valid_move(rows, cols, position):
    return 0 < position[0] <= rows and 0 < position[1] <= cols

def is_valid_position(grid, position):
    return is_valid_move(len(grid), len(grid[0]), position) and grid[position[0] - 1][position[1] - 1] != 'X'

def get_neighbors(position, grid):
    neighbors = [(position[0] - 1, position[1]), (position[0] + 1, position[1]),
                 (position[0], position[1] - 1), (position[0], position[1] + 1)]
    return [neighbor for neighbor in neighbors if is_valid_position(grid, neighbor)]

def euclidean_distance(position1, position2):
    return sqrt((position1[0] - position2[0]) ** 2 + (position1[1] - position2[1]) ** 2)

def manhattan_distance(position1, position2):
    return abs(position1[0] - position2[0]) + abs(position1[1] - position2[1])

def astar(rows, cols, start_pos, end_pos, grid, heuristic):
    if heuristic == 'euclidean':
        distance = euclidean_distance
    elif heuristic == 'manhattan':
        distance = manhattan_distance
    else:
        raise ValueError("Invalid heuristic")

    frontier = []
    heapq.heappush(frontier, Node(start_pos))
    explored = set()

    while frontier:
        node = heapq.heappop(frontier)
        if node.state == end_pos:
            return node
        explored.add(node.state)
        for action in get_neighbors(node.state, grid):
            if action not in explored:
                new_cost = node.path_cost + 1
                h = distance(action, end_pos)
                total_cost = new_cost + h
                child = Node(action, node, path_cost=new_cost)
                heapq.heappush(frontier, child)
                explored.add(action)
    return None

def main():
    if len(sys.argv) < 3 or len(sys.argv) > 4:
        print("Usage: python pathfinder.py [map] [algorithm] [heuristic (optional)]")
        return

    map_filename = sys.argv[1]
    algorithm = sys.argv[2]
    heuristic = sys.argv[3] if len(sys.argv) == 4 else None

    algorithms = {'astar': astar}
    heuristics = {'euclidean', 'manhattan'}

    if algorithm not in algorithms or (algorithm == 'astar' and heuristic not in heuristics):
        print("Invalid algorithm or heuristic.")
        return

    rows, cols, start_pos, end_pos, grid = read_map(map_filename)

    search_algorithm = algorithms[algorithm]

    solution = search_algorithm(rows, cols, start_pos, end_pos, grid, heuristic)

    if solution:
        path = build_path(solution)
        for pos in path:
            grid[pos[0] - 1][pos[1] - 1] = '*'
        print_map(grid)
    else:
        print("null")

def build_path(node):
    path = []
    while node:
        path.append(node.state)
        node = node.parent
    return path[::-1]

if __name__ == "__main__":
    main()