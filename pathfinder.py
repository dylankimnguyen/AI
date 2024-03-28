import sys
import os
import copy
import heapq
from array import *
from collections import deque
from queue import PriorityQueue
from math import sqrt

mapInput = sys.argv[1]
algorithm = sys.argv[2]
if len(sys.argv) > 3:
    heuristic = sys.argv[3]

map = []
xSquares = 0
ySquares = 0

class node:
    def __init__(self, parent, pos, cost):
        self.parent = parent
        self.pos = pos
        self.cost = cost

def sortkey(node):
    return node.cost

def read_goal():
    global start
    global end

    file = open(mapInput,'r')
    list = file.read().splitlines()
    startList = list[1].split(' ')
    start = (int(startList[0]) - 1,int(startList[1]) - 1)
    endList = list[2].split(' ')
    end = (int(endList[0]) - 1,int(endList[1]) - 1)

def read_file():
    global xSquares
    global ySquares
    global start
    global end

    file = open(mapInput,'r')
    list = file.read().splitlines()
    lengthList = list[0].split(' ')
    xSquares = int(lengthList[0])
    ySquares = int(lengthList[1])
    del list[0],list[0],list[0]

    for i in range(len(list)):
        templist = list[i].split(' ')
        for j in range(0, len(templist)):
            if(templist[i] != 'X'):
                templist[i] = int(templist[i])
        map.append(templist)

    return map

def print_array(array):
    for i in range(0, len(array)):
        for j in range(0, len(array)):
            if (j < len(array) - 1): # if not last element in row
                print(array[i][j], end = " ")
            else:
                print(array[i][j], end = '') #if last element in row
        print()

def getChildren(parent, tracker):
    new_children = []
    possibleMoves = [[-1,0],[1,0], [0,-1], [0,1]] #up down left right
    
    for i in range(0, len(possibleMoves)):
        new_pos = possibleMoves[i]
        next_pos = (parent.pos[0] + new_pos[0], parent.pos[1] + new_pos[1])

        if(next_pos[0] == -1 or next_pos[1] == -1 or next_pos[0] >= xSquares or next_pos[1] >= ySquares):
            continue
        elif(map[next_pos[0]][next_pos[1]] == 'X'):
            continue
        elif(tracker[next_pos[0]][next_pos[1]] == True):
            continue
        else:
            new_cost = int(map[next_pos[0]][next_pos[1]]) + parent.cost
            new_node = node(parent, next_pos, new_cost)
            new_children.append(new_node)

    return new_children

def getChildrenAstar(parent, tracker, goal):
    new_children = []
    possibleMoves = [[-1,0],[1,0], [0,-1], [0,1]] #up down left right

    for i in range(0, len(possibleMoves)):
        new_pos = possibleMoves[i]
        next_pos = (parent.pos[0] + new_pos[0], parent.pos[1] + new_pos[1])

        if(next_pos[0] == -1 or next_pos[1] == -1 or next_pos[0] >= xSquares or next_pos[1] >= ySquares):
            continue
        elif(map[next_pos[0]][next_pos[1]] == 'X'):
            continue
        elif(tracker[next_pos[0]][next_pos[1]] == True):
            continue
        else:
            g =  int(map[next_pos[0]][next_pos[1]]) + parent.cost

            if (heuristic == "manhattan"):
                h = abs((next_pos[0] - goal[0])) + abs((next_pos[1] - goal[1]))

            if (heuristic == "euclidean"):
                h = sqrt(((next_pos[0] - goal[0]) ** 2) + ((next_pos[1] - goal[1]) ** 2))

            f = g + h #add cost + heuristic cost
            new_cost = f
            new_node = node(parent, next_pos, new_cost)
            new_children.append(new_node)

    return new_children

def bfs(startPos, endPos):
    map = read_file()
    visited = []

    for i in range (0, xSquares):
        templist = []
        for j in range (0, ySquares):
            templist.append(False)
        visited.append(templist)

    start = node("None", startPos, 0)
    current = start
    queue = deque([])
    visited[start.pos[0]][start.pos[1]] = True
    queue.append(start)

    while queue:
        current = queue.popleft()
        if(current.pos == endPos):
            break

        children = getChildren(current, visited)

        for i in range(0, len(children)):
            visited[children[i].pos[0]][children[i].pos[1]] = True
            queue.append(children[i])

    while current.parent != "None":
        map[current.pos[0]][current.pos[1]] = '*'
        current = current.parent
    map[current.pos[0]][current.pos[1]] = '*'
    print_array(map)

def ucs(startPos, endPos):
    map = read_file()
    visited = []

    for i in range (0, xSquares):
        templist = []
        for j in range (0, ySquares):
            templist.append(False)
        visited.append(templist)

    start = node("None", startPos, 0)
    current = start
    queue = deque([])
    visited[start.pos[0]][start.pos[1]] = True
    queue.append(start)

    found = False

    while queue:
        current = queue.popleft()
        if(current.pos == endPos):
            found = True
            break

        children = getChildren(current, visited)

        children.sort(key=sortkey)

        for i in range(0, len(children)):
            visited[children[i].pos[0]][children[i].pos[1]] = True
            queue.append(children[i])

    if(found == True):
        while current.parent != "None":
            map[current.pos[0]][current.pos[1]] = '*'
            current = current.parent
        map[current.pos[0]][current.pos[1]] = '*'

        print_array(map)

    else:
        print("null")

def astar(startPos, endPos):
    map = read_file()

    visited = []
    for i in range (0, xSquares):
        templist = []
        for j in range (0, ySquares):
            templist.append(False)
        visited.append(templist)

    start = node("None", startPos, 0)
    current = start
    queue = deque([])
    visited[start.pos[0]][start.pos[1]] = True
    queue.append(start)

    found = False

    while queue:
        current = queue.popleft()

        if(current.pos == endPos):
            found = True
            break

        children = getChildrenAstar(current, visited, endPos)
        children.sort(key=sortkey)

        for i in range(0, len(children)):
            visited[children[i].pos[0]][children[i].pos[1]] = True
            queue.append(children[i])

    if(found == True):
        while current.parent != "None":
            map[current.pos[0]][current.pos[1]] = '*'
            current = current.parent
        map[current.pos[0]][current.pos[1]] = '*'

        print_array(map)

    else:
        print("null")

read_goal()

if(algorithm == "bfs"):
    bfs(start),(end)
elif(algorithm == "ucs"):
    ucs((start),(end))
elif(algorithm == "astar"):
    astar((start),(end))
