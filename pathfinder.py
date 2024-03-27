import heapq
from collections import deque
import sys

#def stuctures
class Node:
    def __init__(self, state, path=[], cost=0):
        self.state = state
        self.path = path 
        self.cost = cost  

    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        return self.state == other.state

# need to read map first
def read_map_file(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    map_size = tuple(map(int, lines[0].split()))
    start_position = tuple(map(int, lines[1].split()))
    end_position = tuple(map(int, lines[2].split()))
    map_data = [list(line.split()) for line in lines[3:]]

    return map_size, start_position, end_position, map_data

#algortihms
def bfs_search(map_size, start_position, end_position, map_data):
    fringe = deque([Node(start_position)])
    visited = set()  
    actions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

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
                
                new_path = node.path + [new_position]
                fringe.append(Node(new_position, new_path))

    return "null" 

def ucs_search(map_size, start_position, end_position, map_data):
    fringe = [(0, Node(start_position))]
    visited = set()  
    actions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

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
                new_path = node.path + [new_position]
                heapq.heappush(fringe, (new_cost, Node(new_position, new_path, new_cost)))

    return "null" 

def astar_search(map_size, start_position, end_position, map_data, heuristic):
    fringe = [(0, Node(start_position))]
    visited = set()  
    actions = [(0, -1), (0, 1), (-1, 0), (1, 0)]

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
    if len(sys.argv) != 4:
        print("Usage: python pathfinder.py [map] [algorithm] [heuristic]")
        sys.exit(1)

    map_file = sys.argv[1]
    algorithm = sys.argv[2]
    heuristic = sys.argv[3]

    map_size, start_position, end_position, map_data = read_map_file(map_file)

    if algorithm == 'bfs':
        result = bfs_search(map_size, start_position, end_position, map_data)
    elif algorithm == 'ucs':
        result = ucs_search(map_size, start_position, end_position, map_data)
    elif algorithm == 'astar':
        result = astar_search(map_size, start_position, end_position, map_data, heuristic)
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
            
