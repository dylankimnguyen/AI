import sys
import heapq
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
    neighbors = [(position[0] - 1, position[1]),  # Up
                 (position[0] + 1, position[1]),  # Down
                 (position[0], position[1] - 1),  # Left
                 (position[0], position[1] + 1)]  # Right
    return [neighbor for neighbor in neighbors if is_valid_position(grid, neighbor)]

def bfs(grid, start_pos, end_pos):
    frontier = [Node(start_pos)]
    explored = set()
    while frontier:
        node = frontier.pop(0)
        if node.state == end_pos:
            return node
        explored.add(node.state)
        for action in get_neighbors(node.state, grid):
            if action not in explored:
                child = Node(action, node)
                frontier.append(child)
                explored.add(action)
    return None

def ucs(grid, start_pos, end_pos, heuristic=None):
    frontier = []
    heapq.heappush(frontier, Node(start_pos))
    explored = set()
    while frontier:
        node = heapq.heappop(frontier)
        if node.state == end_pos:
            return node
        explored.add(node.state)
        neighbors = get_neighbors_with_priority(node.state, grid)
        for action in neighbors:
            if action not in explored:
                new_cost = node.path_cost + get_path_cost(grid, node.state, action)
                child = Node(action, node, path_cost=new_cost)
                heapq.heappush(frontier, child)
                explored.add(action)
    return None

def get_neighbors_with_priority(pos, grid):
    neighbors = []
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    for direction in directions:
        new_row = pos[0] + direction[0]
        new_col = pos[1] + direction[1]
        if 0 < new_row <= len(grid) and 0 < new_col <= len(grid[0]):
            neighbors.append((new_row, new_col))
    return neighbors

def astar(grid, start_pos, end_pos, heuristic):
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
                new_cost = node.path_cost + get_path_cost(grid, node.state, action)
                h = distance(action, end_pos)
                total_cost = new_cost + h
                child = Node(action, node, path_cost=new_cost)
                heapq.heappush(frontier, child)
                explored.add(action)
    return None

def get_path_cost(grid, current_pos, next_pos):
    current_elevation = grid[current_pos[0] - 1][current_pos[1] - 1]
    next_elevation = grid[next_pos[0] - 1][next_pos[1] - 1]
    if current_elevation == 'X' or next_elevation == 'X':
        return float('inf')
    current_elevation = int(current_elevation)
    next_elevation = int(next_elevation)
    elevation_diff = abs(next_elevation - current_elevation)
    if next_elevation >= current_elevation:
        return 1 + elevation_diff
    else:
        return 1

def euclidean_distance(position1, position2):
    return sqrt((position1[0] - position2[0]) ** 2 + (position1[1] - position2[1]) ** 2)

def manhattan_distance(position1, position2):
    return abs(position1[0] - position2[0]) + abs(position1[1] - position2[1])

def build_path(node):
    path = []
    while node:
        path.append(node.state)
        node = node.parent
    return path[::-1]

def main():
    if len(sys.argv) < 3 or len(sys.argv) > 4:
        print("Usage: python pathfinder.py [map] [algorithm] [heuristic (optional)]")
        return

    map_filename = sys.argv[1]
    algorithm = sys.argv[2]
    heuristic = sys.argv[3] if len(sys.argv) == 4 else None

    algorithms = {'bfs': bfs, 'ucs': ucs, 'astar': astar}
    heuristics = {'euclidean', 'manhattan'}

    if algorithm not in algorithms or (algorithm == 'astar' and heuristic not in heuristics):
        print("Invalid algorithm or heuristic.")
        return

    rows, cols, start_pos, end_pos, grid = read_map(map_filename)
    search_algorithm = algorithms[algorithm]

    if algorithm == 'bfs':
        solution = search_algorithm(grid, start_pos, end_pos)
    else:
        solution = search_algorithm(grid, start_pos, end_pos, heuristic)

    if solution:
        path = build_path(solution)
        for pos in path:
            grid[pos[0] - 1][pos[1] - 1] = '*'
        print_map(grid)
    else:
        print("null")

if __name__ == "__main__":
    main()
