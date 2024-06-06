import numpy as np
import sys
import math

# k = 11 in this case
k = 11

class Node:
    def __init__(self, point = None, left = None, right = None, dim = None, val=0):
        self.point = point
        self.left = left
        self.right = right
        self.dim = dim
        self.val = val

def read_file(file_path):
    data = np.loadtxt(file_path, skiprows=1)
    features = data[:, :]
    return features

def sort_points(points, split_dim):
    # Rearrange points array based on sorted values
    sorted_points = points[points[:, split_dim].argsort()]

    # numbers of points
    num_points = sorted_points.shape[0]

    # Calculate median index and extract median point
    if num_points % 2 == 0:  # even number of points
        median_idx = num_points // 2 - 1
        median_point1 = sorted_points[median_idx]
        median_point2 = sorted_points[median_idx + 1]
        median_value = (median_point1[split_dim] + median_point2[split_dim]) / 2
        median_point = (median_point1 + median_point2) / 2
    else:  # odd number of points
        median_idx = num_points // 2
        median_point = sorted_points[median_idx]
        median_value = median_point[split_dim]
    # sorted_points, median_value(val),
    return sorted_points, median_point, median_value, median_idx


def build_kd_tree(points, depth):
    # base cases
    # no points
    if points.shape[0] == 0:
        return None

    # one point
    elif points.shape[0] == 1:
        node = Node()
        node.dim = 0
        node.val = 0
        node.point = points[0]
        return node

    else:
        # dim is the split dimension
        split_dim = depth % k
        sorted_points, median_point, median_value, median_idx = sort_points(points, split_dim)

        node = Node()
        # val is the medium value along that dimension
        val = median_value
        node.dim = split_dim
        node.val = val

        left_points = sorted_points[:median_idx + 1]
        right_points = sorted_points[median_idx + 1:]
        node.left = build_kd_tree(left_points, depth + 1)
        node.right = build_kd_tree(right_points, depth + 1)
        return node

# distance
def distance_squared(point1, point2):
    return sum((x - y) ** 2 for x, y in zip(point1[:], point2[:]))


def nearest_point(node, point, depth, best_distance, best=None):
    # base case: get to the leaf node
    if node.left is None and node.right is None:
        distance = distance_squared(node.point, point)
        if distance < best_distance:
            best = node
            best_distance = distance
        return best, best_distance

    # traverse the tree until reach the leaf node
    axis = depth % k
    if point[axis] < node.val:
        next_subtree = node.left
        opposite_subtree = node.right
    else:
        next_subtree = node.right
        opposite_subtree = node.left
    best, best_distance = nearest_point(next_subtree, point, depth + 1, best_distance, best)

    # unwind the tree
    if best_distance >= (point[axis] - node.val) ** 2:
        best, best_distance = nearest_point(opposite_subtree, point, depth + 1, best_distance, best)

    return best, best_distance


def main():

    # parameters from input
    train_file = open(sys.argv[1])
    test_file = open(sys.argv[2])
    dimension = int(sys.argv[3])

    # load files
    train_features = read_file(train_file)
    test_features = read_file(test_file)

    tree = build_kd_tree(train_features, dimension)
    for data in range(test_features.shape[0]):
        point = test_features[data]
        best_point = nearest_point(tree, point, dimension, 10000, best=None)
        print(int(best_point[0].point[11]))


if __name__ == '__main__':
    main()