#test\

def read_map_file(file_path):
    with open (file_path, 'r') as file:
        lines = file.readlines()

    map_size = tuple(map(int, lines[0].split()))
    start_position = tuple(map(int, lines[1].split()))
    end_position = tuple(map(int, lines[2].split()))
    map_data = [list(line.split()) for line in lines[3:]]

    return map_size, start_position, end_position, map_data

def graph_search(problem, fringe):
    closed = set()
    initial_node = make_node(problem.initial_state)
    fringe = insert(initial_node, fringe)

    while True: 
        if is_empty(fringe):
            return "failure"
        
        node =remove_front(fringe)

        if goal_test(problem, state(node)):
            return node
        
        if state(node) not in closed
            closed.add(state(node))
            children = expand(node, problem)
            for child in children:
                fringe = insert(child, fringe)

        
