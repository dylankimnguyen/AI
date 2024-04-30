class Node:
    def __init__(self, d, val, point):
        self.d = d
        self.val = val
        self.point = point
        self.left = None
        self.right = None

def build_kd_tree(P, D):
    M = len(P[0])  # Number of dimensions
    if not P:
        return None
    elif len(P) == 1:
        return Node(D % M, P[0][D % M], P[0])
    else:
        d = D % M
        P.sort(key=lambda x: x[d])  # Sort points along the current dimension
        median_index = len(P) // 2
        val = P[median_index][d]
        node = Node(d, val, P[median_index])
        node.left = build_kd_tree(P[:median_index], D + 1)
        node.right = build_kd_tree(P[median_index + 1:], D + 1)
        return node
