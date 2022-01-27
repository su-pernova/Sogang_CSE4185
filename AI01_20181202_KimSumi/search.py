# --------- Search the Maze Solution --------- #
def search(maze, func):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_four_circles": astar_four_circles,
        "astar_many_circles": astar_many_circles
    }.get(func)(maze)


# -------------------- Stage 01: One circle - BFS Algorithm ------------------------ #
# Node class for BFS Search
class BFS_Node:
    def __init__(self,parent,location):
        self.parent = parent
        self.location = location
        self.mazeTable = []

# BFS Search Function
def bfs(maze):
    start_point = maze.startPoint()
    path = []

    # 방문한 위치 check를 위한 mazeTable
    mazeSize = maze.getDimensions()
    mazeTable = [[0 for col in range(mazeSize[1])] for row in range(mazeSize[0])]
    
    # bfs_tree for bfs search
    bfs_tree = []
    bfs_tree.append(BFS_Node(None,start_point))
    bfs_tree[0].mazeTable = mazeTable.copy()

    # bfs_queue for bfs search
    bfs_queue = []

    # initialize root node
    next_pos = maze.neighborPoints(start_point[0],start_point[1])
    bfs_tree[0].mazeTable[start_point[0]][start_point[1]] = 1

    for i in next_pos:
        bfs_queue.append(BFS_Node(bfs_tree[0],i))
        bfs_tree.append(BFS_Node(bfs_tree[0],i))
        bfs_queue[-1].mazeTable = bfs_queue[-1].parent.mazeTable.copy()
        bfs_queue[-1].mazeTable[i[0]][i[1]] = 1
        if i == maze.circlePoints()[0]:
            path.append(start_point)
            path.append(i)
            return path

    # bfs search start
    break_flag = 0
    while 1:
        next_pos = maze.neighborPoints((bfs_queue[0].location)[0],(bfs_queue[0].location)[1])

        for i in next_pos:
            if (bfs_queue[0].mazeTable[i[0]][i[1]]) == 0:
                bfs_queue.append(BFS_Node(bfs_queue[0],i))
                bfs_tree.append(BFS_Node(bfs_queue[0],i))
                bfs_queue[-1].mazeTable = bfs_queue[-1].parent.mazeTable.copy()
                bfs_queue[-1].mazeTable[i[0]][i[1]] = 1
                if i == maze.circlePoints()[0]: 
                    break_flag = 1
                    break
        
        if break_flag == 1: break
        del bfs_queue[0]

    # make path
    current_node = bfs_tree[-1]
    while 1:
        path.insert(0, current_node.location)
        if current_node.location == start_point: break
        current_node = current_node.parent
            
    # return result
    return path

# -------------------- Stage 01: One circle - A* Algorithm ------------------------ #
# Node Class for A* Search
class As_Node:
    def __init__(self,parent,location):
        self.parent = parent
        self.location = location
        self.obj = []
        self.goals = []
        self.mazeTable = []
        self.mst_nodes = []

        # F = G+H
        self.f = 0
        self.g = 0
        self.h = 0

# Heuristic Function
def manhatten_dist(p1,p2):
    return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])

# Astar Search Function
def astar(maze):
    start_point = maze.startPoint()
    end_point = maze.circlePoints()[0]
    path = []

    # 방문한 위치 check를 위한 mazeTable 생성
    mazeSize = maze.getDimensions()
    mazeTable = [[0 for col in range(mazeSize[1])] for row in range(mazeSize[0])]

    # As_tree for Astar search
    Ast_tree = []

    # initialize root node
    Ast_tree.append(As_Node(None,start_point))
    Ast_tree[0].obj = maze.neighborPoints(start_point[0],start_point[1])
    Ast_tree[0].mazeTable = mazeTable.copy()
    Ast_tree[0].mazeTable[start_point[0]][start_point[1]] = 1
    Ast_tree[0].g = 0
    Ast_tree[0].h = manhatten_dist(start_point,end_point)
    Ast_tree[0].f = Ast_tree[0].g + Ast_tree[0].h

    # make open_list for Astar search
    open_list = []
    list_idx = 0
    for i in Ast_tree[0].obj:
        if i == end_point:
            path = [start_point,end_point]
            return path
        Ast_tree.append(As_Node(Ast_tree[0],i))
        open_list.append(As_Node(Ast_tree[0],i))
        open_list[list_idx].obj = maze.neighborPoints((open_list[list_idx].location)[0],(open_list[list_idx].location)[1])
        open_list[list_idx].mazeTable = open_list[list_idx].parent.mazeTable.copy()
        open_list[list_idx].mazeTable[i[0]][i[1]] = 1
        open_list[list_idx].g = open_list[list_idx].parent.g + 1
        open_list[list_idx].h = manhatten_dist(open_list[list_idx].location,end_point)
        open_list[list_idx].f = open_list[list_idx].g + open_list[list_idx].h
        list_idx += 1
    
    # start Astar search
    search_flag = 0
    while 1:
        # search minimum f
        min_f = open_list[0].f
        for i in open_list:
            if i.f < min_f: min_f = i.f

        # update open_list
        loop = len(open_list)
        for i in range(loop):
            if open_list[i].f == min_f:
                for j in open_list[i].obj:
                    if open_list[i].mazeTable[j[0]][j[1]] == 0:
                        Ast_tree.append(As_Node(open_list[i],j))
                        open_list.insert(i+1,As_Node(open_list[i],j))

                        # if goal found : end search
                        if j == maze.circlePoints()[0]:
                            search_flag = 1
                            break

                        list_idx = i+1
                        open_list[list_idx].obj = maze.neighborPoints((open_list[list_idx].location)[0],(open_list[list_idx].location)[1])
                        open_list[list_idx].mazeTable = open_list[list_idx].parent.mazeTable.copy()
                        open_list[list_idx].mazeTable[j[0]][j[1]] = 1
                        open_list[list_idx].g = open_list[list_idx].parent.g + 1
                        open_list[list_idx].h = manhatten_dist(open_list[list_idx].location,end_point)
                        open_list[list_idx].f = open_list[list_idx].g + open_list[list_idx].h
                if search_flag == 1: break
                open_list[i].f = -1
                break

        if search_flag == 1: break

        while 1:
            break_flag = 1
            for i in open_list:
                if i.f == -1: 
                    break_flag = 0
                    open_list.remove(i)
                    break
            if break_flag == 1: break

    # make path
    current_node = Ast_tree[-1]
    while 1:
        path.insert(0, current_node.location)
        if current_node.location == start_point: break
        current_node = current_node.parent

    # return result
    return path


# -------------------- Stage 02: Four circles - A* Algorithm  ------------------------ #
# mst node class
class MST_Node:
    def __init__(self,set_num,location):
        self.set_num = set_num
        self.location = location

# mst edge class
class Edge:
    def __init__(self,nodes,weight):
        self.nodes = nodes
        self.weight = weight

# Minimum Spanning Tree Function(Kruskal)
def mst(goal_nodes,nodes,dist_table):
    mst_nodes = [];mst_edges = []
    cost_sum = 0; set_num = 0
    goals = goal_nodes.copy()

    # 1) make nodes
    for i in goals:
        mst_nodes.append(MST_Node(set_num,i))
        set_num += 1

    # 2) make edges
    for i in nodes:
        for j in goals:
            if i != j:
                for k in mst_nodes:
                    if k.location == i: p1 = k
                for k in mst_nodes:
                    if k.location == j: p2 = k
                dist = dist_table[(i,j)]
                mst_edges.append(Edge([p1,p2],dist))
        goals.remove(i)

    # 3) make mst
    break_flag = 1
    while break_flag:
        min_edge = mst_edges[0]
        for i in mst_edges:
            if i.weight < min_edge.weight:
                min_edge = i

        check1 = 0 # set shoud be changed
        check2 = 0 # change into this set
        if min_edge.nodes[0].set_num != min_edge.nodes[1].set_num:
            cost_sum += min_edge.weight
            if min_edge.nodes[0].set_num < min_edge.nodes[1].set_num:
                check1 = min_edge.nodes[1].set_num
                check2 = min_edge.nodes[0].set_num
                min_edge.nodes[1].set_num = min_edge.nodes[0].set_num
            else:
                check1 = min_edge.nodes[0].set_num
                check2 = min_edge.nodes[1].set_num
                min_edge.nodes[0].set_num = min_edge.nodes[1].set_num
            for i in mst_nodes:
                if i.set_num == check1: i.set_num = check2
        mst_edges.remove(min_edge)
        
        break_flag = 0
        union = mst_nodes[0].set_num
        for i in mst_nodes:
            if i.set_num != union:
                break_flag = 1

    return cost_sum

# Astar Search Function
def As_search(start_point,goal,maze):
    path=[]
    mazeSize = maze.getDimensions()
    mazeTable = [[0 for col in range(mazeSize[1])] for row in range(mazeSize[0])]
    mazeTable[start_point[0]][start_point[1]] = 1

    Ast_tree = []
    Ast_tree.append(As_Node(None,start_point))
    Ast_tree[0].obj = maze.neighborPoints(start_point[0],start_point[1])
    Ast_tree[0].g = 0
    Ast_tree[0].h = manhatten_dist(start_point,goal)
    Ast_tree[0].f = Ast_tree[0].g + Ast_tree[0].h

    open_list = []
    list_idx = 0
    for i in Ast_tree[0].obj:
        if i == goal:
            path = [start_point,goal]
            return path
        mazeTable[i[0]][i[1]] = 1
        Ast_tree.append(As_Node(Ast_tree[0],i))
        open_list.append(As_Node(Ast_tree[0],i))
        open_list[list_idx].obj = maze.neighborPoints((open_list[list_idx].location)[0],(open_list[list_idx].location)[1])
        open_list[list_idx].g = open_list[list_idx].parent.g + 1
        open_list[list_idx].h = manhatten_dist(open_list[list_idx].location,goal)
        open_list[list_idx].f = open_list[list_idx].g + open_list[list_idx].h
        list_idx += 1
    
    search_flag = 0
    while 1:
        min_f = open_list[0].f
        for i in open_list:
            if i.f < min_f: min_f = i.f
        loop = len(open_list)
        for i in range(loop):
            if open_list[i].f == min_f:
                for j in open_list[i].obj:
                    if mazeTable[j[0]][j[1]] == 0:
                        mazeTable[j[0]][j[1]] = 1
                        Ast_tree.append(As_Node(open_list[i],j))
                        open_list.insert(i+1,As_Node(open_list[i],j))
                        if j == goal:
                            search_flag = 1
                            break
                        list_idx = i+1
                        open_list[list_idx].obj = maze.neighborPoints((open_list[list_idx].location)[0],(open_list[list_idx].location)[1])
                        open_list[list_idx].g = open_list[list_idx].parent.g + 1
                        open_list[list_idx].h = manhatten_dist(open_list[list_idx].location,goal)
                        open_list[list_idx].f = open_list[list_idx].g + open_list[list_idx].h
                if search_flag == 1: break
                open_list[i].f = -1
                break
        if search_flag == 1: break
        while 1:
            break_flag = 1
            for i in open_list:
                if i.f == -1: 
                    break_flag = 0
                    open_list.remove(i)
            if break_flag == 1: break

    current_node = Ast_tree[-1]
    while 1:
        path.insert(0, current_node.location)
        if current_node.location == start_point: break
        current_node = current_node.parent
    
    return path

# stage2_heuristic function
def stage2_heuristic(goal_nodes,nodes,dist_table):
    return mst(goal_nodes,nodes,dist_table)

# astar four circles search function
def astar_four_circles(maze):
    start_point = maze.startPoint()
    end_points=maze.circlePoints()
    end_points.sort()
    path=[start_point]

    goal_nodes = end_points.copy()
    goal_nodes.append(start_point)

    # distance table for better time complexity
    dist_table = {}
    for i in goal_nodes:
        for j in goal_nodes:
            if i!=j:
                dist_table[(i,j)] = len(As_search(i,j,maze))-1

    # make astar tree
    Ast_tree = []
    Ast_tree.append(As_Node(None,start_point))
    Ast_tree[0].obj = goal_nodes.copy()
    Ast_tree[0].obj.remove(start_point)
    Ast_tree[0].mst_nodes = [start_point]
    Ast_tree[0].g = 0
    Ast_tree[0].h = stage2_heuristic(goal_nodes,Ast_tree[0].mst_nodes,dist_table)
    Ast_tree[0].f = Ast_tree[0].g + Ast_tree[0].h

    # make open_list
    open_list = []; list_idx = 0
    for i in Ast_tree[0].obj:
        Ast_tree.append(As_Node(Ast_tree[0],i))
        open_list.append(As_Node(Ast_tree[0],i))
        open_list[list_idx].obj = open_list[list_idx].parent.obj.copy()
        open_list[list_idx].obj.remove(i)
        open_list[list_idx].mst_nodes = open_list[list_idx].parent.mst_nodes.copy()
        open_list[list_idx].mst_nodes.append(i)
        open_list[list_idx].g = open_list[list_idx].parent.g + dist_table[(open_list[list_idx].parent.location,i)]
        open_list[list_idx].h = stage2_heuristic(goal_nodes,Ast_tree[0].mst_nodes,dist_table)
        open_list[list_idx].f = open_list[list_idx].g + open_list[list_idx].h
        list_idx += 1

    # start astar search
    search_flag = 0
    while 1:
        min_f = open_list[0].f
        for i in open_list:
            if i.f < min_f: min_f = i.f
        
        loop = len(open_list)
        for i in range(loop):
            if open_list[i].f == min_f:

                if len(open_list[i].obj) == 0:
                    current_node = open_list[i]
                    search_flag = 1
                    break

                for j in open_list[i].obj:
                        Ast_tree.append(As_Node(open_list[i],j))
                        open_list.insert(i+1,As_Node(open_list[i],j))

                        list_idx = i+1
                        open_list[list_idx].obj = open_list[list_idx].parent.obj.copy()
                        open_list[list_idx].obj.remove(j)
                        open_list[list_idx].mst_nodes = open_list[list_idx].parent.mst_nodes.copy()
                        open_list[list_idx].mst_nodes.append(j)
                        open_list[list_idx].g = open_list[list_idx].parent.g + dist_table[(open_list[list_idx].parent.location,j)]
                        open_list[list_idx].h = stage2_heuristic(goal_nodes,open_list[list_idx].mst_nodes,dist_table)
                        open_list[list_idx].f = open_list[list_idx].g + open_list[list_idx].h
                if search_flag == 1: break
                open_list[i].f = -1
                break

        if search_flag == 1: break

        while 1:
            break_flag = 1
            for i in open_list:
                if i.f == -1: 
                    break_flag = 0
                    open_list.remove(i)
            if break_flag == 1: break

    # make goal order
    goal_order = []
    while 1:
        goal_order.insert(0, current_node.location)
        if current_node.location == start_point: break
        current_node = current_node.parent

    # make path
    for i in range(len(goal_order)-1):
        this_path = As_search(goal_order[i],goal_order[i+1],maze)
        del this_path[0]
        path += this_path

    return path


# -------------------- Stage 03: Many circles - A* Algorithm -------------------- #
def astar_many_circles(maze):
    start_point = maze.startPoint()
    end_points= maze.circlePoints()
    end_points.sort()
    mazeSize = maze.getDimensions()
    path=[]

    # distance table for better time complexity
    dist_table = {}
    for i in end_points:
        dist_table[(start_point,i)] = len(As_search(start_point,i,maze))-1
    for i in end_points:
        for j in end_points:
            if  i!= j:
                if (j,i) in dist_table:
                    dist_table[(i,j)] = dist_table[(j,i)]
                else:
                    dist_table[(i,j)] = len(As_search(i,j,maze))-1

    # Minimum Spanning Tree Function(Prim)
    def mst(start_point,end_points):
        visited = [start_point]
        goals = end_points.copy()
        cost_sum = 0

        while len(goals) > 0:
            min_goal = goals[0]
            if (visited[0],goals[0]) in dist_table:
                min_dist = dist_table[(visited[0],goals[0])]
            else: 
                dist_table[(visited[0],goals[0])] = len(As_search(visited[0],goals[0],maze))-1
                dist_table[(goals[0],visited[0])] = dist_table[(visited[0],goals[0])]
                min_dist = dist_table[(visited[0],goals[0])]

            for i in visited:
                for j in goals:
                    if (i,j) in dist_table:
                        dist = dist_table[(i,j)]
                    else: 
                        dist_table[(i,j)] = len(As_search(i,j,maze))-1
                        dist_table[(j,i)] = dist_table[(i,j)]
                        dist = dist_table[(i,j)]
                    if dist < min_dist:
                        min_dist = dist
                        min_goal = j

            visited.append(min_goal)
            goals.remove(min_goal)
            cost_sum += min_dist

        cost_sum += 1
        return cost_sum

    # Stage3 Heuristic Function
    def stage3_heuristic(x,node_goals):
        h = 0
        if (x,node_goals[0]) in dist_table:
                min_dist = dist_table[(x,node_goals[0])]
        else: 
            dist_table[(x,node_goals[0])] = len(As_search(x,node_goals[0],maze))-1
            dist_table[(node_goals[0],x)] = dist_table[(x,node_goals[0])]
            min_dist = dist_table[(x,node_goals[0])]

        for i in node_goals:
            if (x,i) in dist_table:
                dist = dist_table[(x,i)]
            else:
                dist_table[(x,i)] = len(As_search(x,i,maze))-1
                dist_table[(i,x)] = dist_table[(x,i)]
            dist = dist_table[(x,i)]
            if dist < min_dist:
                min_dist = dist

        h += min_dist
        h += mst(x,node_goals)
        if mazeSize[1] > 45:
            h *= 2.4

        return h

    # AMC search를 위한 AMC_tree 생성
    AMC_tree = []

    # 첫번째 노드 AMC_tree[0] initialize
    AMC_tree.append(As_Node(None,start_point))
    AMC_tree[0].mazeTable = [[0 for col in range(mazeSize[1])] for row in range(mazeSize[0])]
    (AMC_tree[0].mazeTable)[start_point[0]][start_point[1]] = 1
    AMC_tree[0].obj = maze.neighborPoints(start_point[0],start_point[1])
    AMC_tree[0].goals = end_points.copy()
    AMC_tree[0].g = 0
    AMC_tree[0].h = stage3_heuristic(start_point,AMC_tree[0].goals)
    AMC_tree[0].f = AMC_tree[0].g + AMC_tree[0].h
    
    # AMC search를 위한 열린 목록 생성
    open_list = []
    list_idx = 0
    for i in AMC_tree[0].obj:
        AMC_tree.append(As_Node(AMC_tree[0],i))
        open_list.append(As_Node(AMC_tree[0],i))

        open_list[list_idx].mazeTable = open_list[list_idx].parent.mazeTable.copy()
        (open_list[list_idx].mazeTable)[i[0]][i[1]] = 1

        open_list[list_idx].obj = maze.neighborPoints((open_list[list_idx].location)[0],(open_list[list_idx].location)[1])
        
        open_list[list_idx].goals = open_list[list_idx].parent.goals.copy()
        if i in open_list[list_idx].goals:
            open_list[list_idx].goals.remove(i)
        open_list[list_idx].g = open_list[list_idx].parent.g + 1
        open_list[list_idx].h = stage3_heuristic(open_list[list_idx].location,open_list[list_idx].goals)
        open_list[list_idx].f = open_list[list_idx].g + open_list[list_idx].h
        list_idx += 1

    # AMC search 시작, goal 방문시 search 종료
    search_flag = 0
    buffer = 0
    while 1:
        min_f = open_list[0].f
        for i in open_list:
            if i.f < min_f: min_f = i.f
    
        # update open_list
        loop = len(open_list)
        for i in range(loop):
            if open_list[i].f == min_f:
                for j in open_list[i].obj:
                    if (open_list[i].mazeTable)[j[0]][j[1]]==0:
                        AMC_tree.append(As_Node(open_list[i],j))
                        open_list.insert(i+1,As_Node(open_list[i],j))

                        list_idx = i+1
                        open_list[list_idx].mazeTable = open_list[list_idx].parent.mazeTable.copy()
                        (open_list[list_idx].mazeTable)[j[0]][j[1]] = 1

                        open_list[list_idx].goals = open_list[list_idx].parent.goals.copy()
                        if j in open_list[list_idx].goals:
                            open_list[list_idx].goals.remove(j)
                            open_list[list_idx].mazeTable = [[0 for col in range(mazeSize[1])] for row in range(mazeSize[0])]
                            (open_list[list_idx].mazeTable)[j[0]][j[1]] = 1
                        if len(open_list[list_idx].goals) == 0:
                            search_flag = 1
                            break
                        
                        open_list[list_idx].obj = maze.neighborPoints((open_list[list_idx].location)[0],(open_list[list_idx].location)[1])
                        open_list[list_idx].g = open_list[list_idx].parent.g + 1
                        open_list[list_idx].h = stage3_heuristic(open_list[list_idx].location,open_list[list_idx].goals)
                        open_list[list_idx].f = open_list[list_idx].g + open_list[list_idx].h
                if search_flag == 1: break
                open_list[i].f = -1
                break

        if search_flag == 1: break

        while 1:
            break_flag = 1
            for i in open_list:
                if i.f == -1: 
                    break_flag = 0
                    open_list.remove(i)
            if break_flag == 1: break

    # make path
    current_node = AMC_tree[-1]
    goals = end_points.copy()
    while 1:
        path.insert(0, current_node.location)
        if current_node.location in goals:
            goals.remove(current_node.location)
        if (len(goals)==0) and (current_node.location==start_point): break
        current_node = current_node.parent
        
    # return result
    return path