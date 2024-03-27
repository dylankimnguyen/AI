#test\
function GRAPH-SEARCH (problem, fringe) returns a solution, or failure
    closed <- an empty set
    fringe <- INSERT(MAKE-NODE(INITIAL-STATE[problem]), fringe)
    loop do
        if fringe is empty then return failure
        node <- REMOVE-FRONT(fringe)
        if GOAL-TEST(problem, STATE[node]) then return node
        if STATE[node] is not in closed then
            add STATE[node] to closed
            fringe <- INSERTALL(EXPAND(node, problem), fringe)
    end