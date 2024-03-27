import heapq
from collections import deque
import sys

# Define a Node class to represent nodes in the search tree
class Node:
    def __init__(self, state, path=[], cost=0):
        self.state = state
        self.path = path  # Path from the start node to this node
        self.cost = cost  # Cost of the path from the start node to this node

    # Define comparison methods for heapq
    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return self.state == other.state

# Read map file and return map size, start position, end position, and map data
def read_map_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    map_size = tuple(map(int, lines[0].split()))
    start_position = tuple(map(int, lines[1].split()))
    end_position = tuple(map(int, lines[2].split()))
    map_data = [list(line.split()) for line in lines[3:]]

    return map_size, start_position, end_position, map_data

# Breadth First Search (BFS) algorithm
def bfs_search(map_size, start_position, end_position, map_data):

    fringe = deque([Node(start_position)])
    visited = set()  
    actions = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # Down, Right, Up, Left

    while fringe:
        node = fringe.popleft()  
        position = node.state

        if position == end_position:  
            return node.path

        if position in visited:  
            continue

        visited.add(position)

        for action in actions:
            new_position = (position[0] + action[0], position[1] + action[1])
            if (0 < new_position[0] <= map_size[0] and
                0 < new_position[1] <= map_size[1] and
                map_data[new_position[0] - 1][new_position[1] - 1] != 'X'):
        
                new_path = node.path + [new_position]  # Update the path
                fringe.append(Node(new_position, new_path))  # Append new node with updated path


    return "null"

def ucs_search(map_size, start_position, end_position, map_data):
    opened = [(0, Node(start_position))]
    closed = set()
    node_dict = {start_position: 0}
    actions = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # Down, Right, Up, Left

    while opened:
        _, selected_node = heapq.heappop(opened)
        position = selected_node.state

        if position == end_position:
            return selected_node.path

        closed.add(position)

        for action in actions:
            new_position = (position[0] + action[0], position[1] + action[1])
            if (0 < new_position[0] <= map_size[0] and
                0 < new_position[1] <= map_size[1] and
                map_data[new_position[0] - 1][new_position[1] - 1] != 'X'):

                new_cost = selected_node.cost + 1
                new_path = selected_node.path + [new_position]

                if new_position not in closed:
                    if new_position not in node_dict or new_cost < node_dict[new_position]:
                        node_dict[new_position] = new_cost
                        heapq.heappush(opened, (new_cost, Node(new_position, new_path, new_cost)))
                    elif new_cost == node_dict[new_position]:  # If cost is equal, prefer downward movement
                        heapq.heappush(opened, (new_cost, Node(new_position, new_path, new_cost)))

    return None  # Return None instead of "null" for consistency



# A* Search algorithm
def astar_search(map_size, start_position, end_position, map_data, heuristic):
    fringe = [(0, Node(start_position))]
    visited = set()  
    actions = [(1, 0), (0, 1), (-1, 0), (0, -1)]

    while fringe:
        _, node = heapq.heappop(fringe)  
        position = node.state

        if position == end_position:  
            return node.path

        if position in visited:  
            continue

        visited.add(position)

        for action in actions:
            new_position = (position[0] + action[0], position[1] + action[1])
            if (0 < new_position[0] <= map_size[0] and
                0 < new_position[1] <= map_size[1] and
                map_data[new_position[0] - 1][new_position[1] - 1] != 'X'):
                
                new_cost = node.cost + 1  
                if heuristic == 'euclidean':
                    heuristic_cost = ((new_position[0] - end_position[0])**2 + 
                                      (new_position[1] - end_position[1])**2)**0.5
                elif heuristic == 'manhattan':
                    heuristic_cost = abs(new_position[0] - end_position[0]) + abs(new_position[1] - end_position[1])
                
                new_path = node.path + [new_position]
                heapq.heappush(fringe, (new_cost + heuristic_cost, Node(new_position, new_path, new_cost)))

    return "null"  

if __name__ == "__main__":
    if len(sys.argv) < 2 or len(sys.argv) > 5:
        print("Usage: python pathfinder.py [map] [algorithm] [heuristic (optional)]")
        sys.exit(1)

    map_file = sys.argv[1]
    algorithm = sys.argv[2]
    heuristic = sys.argv[3] if len(sys.argv) == 4 else None

    map_size, start_position, end_position, map_data = read_map_file(map_file)

    if algorithm == 'bfs':
        result = bfs_search(map_size, start_position, end_position, map_data)
    elif algorithm == 'ucs':
        result = ucs_search(map_size, start_position, end_position, map_data)
    elif algorithm == 'astar':
        if heuristic:
            result = astar_search(map_size, start_position, end_position, map_data, heuristic)
        else:
            print("Error: A* algorithm requires a heuristic.")
            sys.exit(1)
    else:
        print("Invalid algorithm. Choose from 'bfs', 'ucs', or 'astar'.")
        sys.exit(1)

    if result == "null":
        print("null")
    else:
        for i in range(1, map_size[0] + 1):
            for j in range(1, map_size[1] + 1):
                if (i, j) == start_position:
                    print("*", end=" ")
                elif (i, j) in result:
                    print("*", end=" ")
                else:
                    print(map_data[i - 1][j - 1], end=" ")
            print()
