import math
import copy
from Map import Point
import matplotlib.pyplot as plt


def away_free_space(start,obstacles,result):
    print("Hello, let us away the left free space ")


    # 得到周围的路径点， 存储在 points 中
    points = []  # 存储free 点
    get_around_path_position(obstacles,result,points)

    # 将路径组合起来
    way_points = copy.deepcopy(points)
    way_x=[]
    way_y=[]
    while len(way_points)!=0:
        # 得到  points 点中距离 start 点最近的点
        point = get_nearest_piont(start,way_points)
        # 计算从当前点到最近点的路径
        plan_result = get_path_to_nearest_point(start,point,obstacles)
        if not plan_result["found"]:
            print("路径规划失败:", plan_result["reason"], "start=", start, "goal=", point)
            way_points.remove(point)
            continue
        path_points = plan_result["path"]
        rx = [p[0] for p in path_points]
        ry = [p[1] for p in path_points]
        print(rx, ry)
        for i in range(len(rx)):
            way_x.append(rx[i])
            way_y.append(ry[i])
        start = point
        way_points.remove(start)
    return way_x,way_y


# 创建在 dijikstra 中使用的节点
class Node:
    def __init__(self,x,y,g_cost,h_cost,parent_index):
        self.x = x
        self.y = y
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.parant_index = parent_index

    @property
    def cost(self):
        return self.g_cost + self.h_cost


# 计算从当前点到最近点的路径,使用 A* 算法
def get_path_to_nearest_point(start,goal,obstacles,map_bounds=None,valid_mask=None):
    open_set={}
    close_set={}
    obstacle_set = set(obstacles)
    start_node = Node(start[0],start[1], 0, 0, (-1,-1) )
    goal_node = Node(goal[0], goal[1], 0, 0, (-1, -1) )
    open_set[(start_node.x,start_node.y)] = start_node

    motion = [ [1,0,1],[0,1,1],[-1,0,1],[0,-1,1]]
               # [-1,-1,math.sqrt(2)],[-1,1,math.sqrt(2)],[1,-1,math.sqrt(2)],[1,1,math.sqrt(2)]]

    if start in obstacle_set or goal in obstacle_set:
        print("[A*] 起点或终点位于障碍物中: start=", start, "goal=", goal)
        return {"found": False, "path": [], "reason": "start_or_goal_in_obstacle"}

    if map_bounds is None and valid_mask is None:
        all_points = list(obstacle_set) + [start, goal]
        x_values = [p[0] for p in all_points]
        y_values = [p[1] for p in all_points]
        map_bounds = (min(x_values), max(x_values), min(y_values), max(y_values))

    def is_valid_position(pos):
        x, y = pos
        if map_bounds is not None:
            min_x, max_x, min_y, max_y = map_bounds
            if x < min_x or x > max_x or y < min_y or y > max_y:
                return False
        if valid_mask is not None and not valid_mask(pos):
            return False
        return True

    if not is_valid_position(start) or not is_valid_position(goal):
        print("[A*] 起点或终点越界: start=", start, "goal=", goal, "bounds=", map_bounds)
        return {"found": False, "path": [], "reason": "start_or_goal_out_of_bounds"}

    while 1:
        if not open_set:
            return {"found": False, "path": [], "reason": "open_set_exhausted"}
        c_id = min(open_set, key = lambda o: open_set[o].cost)
        current = open_set[c_id]
        # 判断是否是终点
        if current.x == goal_node.x and current.y == goal_node.y:
            print("Find goal")
            print('from'+str(start)+'to'+str(goal)+'is:')
            goal_node.parant_index = current.parant_index
            goal_node.g_cost = current.g_cost
            goal_node.h_cost = current.h_cost
            # close_set[c_id] = current

            # print("the key-value of close set")
            # print(close_set)
            rx, ry = final_path(goal_node, close_set)
            # 输出从 start->goal的路径
            rx = list(reversed(rx))
            ry = list(reversed(ry))
            return {"found": True, "path": list(zip(rx, ry)), "reason": "ok"}

        del open_set[c_id]
        close_set[c_id] = current

        for move_x,move_y,move_cost in motion:
            next_pos = (current.x+move_x,current.y+move_y)
            if not is_valid_position(next_pos):
                continue
            node = Node(
                next_pos[0],
                next_pos[1],
                current.g_cost + move_cost,
                ((next_pos[0]-goal_node.x)**2 + (next_pos[1]-goal_node.y)**2 )**0.5,
                (current.x,current.y)
            )
            n_id = (current.x+move_x,current.y+move_y)
            # print(n_id)
            if n_id in close_set:
                continue
            if n_id in obstacle_set:
                continue
            if n_id not in open_set:
                open_set[n_id] = node
            else:
                if open_set[n_id].cost >= node.cost:
                    open_set[n_id].g_cost = node.g_cost
                    open_set[n_id].h_cost = node.h_cost
                    open_set[n_id].parant_index = node.parant_index



# 回溯最小路径
def final_path(goal_node,close_set):

    rx,ry = [goal_node.x],[goal_node.y]
    parent_index = goal_node.parant_index

    while parent_index != (-1,-1):
        n = close_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parant_index
    return rx,ry

# 判断是否在 open_list 中
def n_exist_in_open_list(i, j, open_list):
    for k in range(len(open_list)):
        if i == open_list[k].x and j == open_list[k].y:
            return 0
    return 1

# 得到  points 点中距离 start 点最近的点
def get_nearest_piont(start,way_points):
    dis_min = math.sqrt((way_points[0][0] - start[0])**2+(way_points[0][1] - start[1])**2)
    id = 0
    for i in range(0,len(way_points)):
        dis = math.sqrt((way_points[i][0] -start[0]) **2 + (way_points[i][1] - start[1]) **2)
        if (dis < dis_min):
            dis_min = dis
            id = i
            if dis_min == 1:
                return (way_points[i][0],way_points[i][1])
    return (way_points[id][0],way_points[id][1])


# 得到周围的路径点
def get_around_path_position(obstacles,result,points):
    # for n in range(1,200):
    #     print(n)
    #     for i in range(0,n+1):
    #         if(  ((start[0]+1,start[1]+i) not in obstacles) and \
    #                 ( (start[0]+1,start[1]+i) not in result) and \
    #                 ((start[0] + 1, start[1] + i) not in points) and \
    #                 (distance(start[0], start[1] + i, obstacles) < 2) and \
    #                 ( distance(start[0],start[1]+i,result) < 2 ) ):
    #             return (start[0]+1,start[1]+i)
    #         elif (((start[0] - i, start[1] + 1) not in obstacles) and \
    #                  ((start[0] - i, start[1] + 1) not in result) and \
    #               ((start[0] -i, start[1] + 1) not in points) and \
    #               (distance(start[0] - i, start[1] + 1, obstacles) < 2) and \
    #               (distance(start[0] - i, start[1] + 1, result) < 2 )):
    #             return (start[0] - i, start[1] + 1)
    #         elif ( ((start[0] - 1, start[1] - i) not in obstacles) and \
    #                  ((start[0] - 1, start[1] - i) not in result) and \
    #                ((start[0] - 1, start[1] - i) not in points) and \
    #                (distance(start[0] - 1, start[1] - i, obstacles) < 2) and \
    #                (distance(start[0] - 1, start[1] - i, result) < 2 ) ):
    #             return (start[0] - 1, start[1] - i)
    #         elif ( ((start[0] + i, start[1] - 1) not in obstacles) and \
    #                  ((start[0] + i, start[1] - 1) not in result) and \
    #                ((start[0] + i, start[1] - 1) not in points) and \
    #                (distance(start[0] + i, start[1] - 1, obstacles) < 2) and \
    #                (distance(start[0] + i, start[1] - 1, result) < 2 ) ):
    #             return (start[0] + i, start[1] - 1)
    #         else:
    #             continue
    # return (None,None)

# 计算 （x,y） 到 result 的最短距离
    x_min = result[0][0]
    x_max = result[0][0]
    y_min = result[0][1]
    y_max = result[0][1]
    for i in range(len(result)):
        if(x_min>result[i][0]):
            x_min=result[i][0]
        if(x_max<result[i][0]):
            x_max=result[i][0]
        if(y_min>result[i][1]):
            y_min=result[i][1]
        if(y_max<result[i][1]):
            y_max=result[i][1]
    # print(x_min,x_max,y_min,y_max)
    for i in range(x_min-1,x_max+1):
        for j in range(y_min-1, y_max+1):
            if ( distance(i,j,obstacles)==1 and distance(i,j,result)<2 and \
                    ((i, j) not in obstacles) and \
                    ((i, j) not in result)):
                points.append((i,j))

def distance(x,y,res):
    dis_min = 50
    for i in range(len(res)):
        dis = math.sqrt( (res[i][0]-x)*(res[i][0]-x) + (res[i][1]-y)*(res[i][1]-y))
        if ( dis < dis_min ):
            dis_min = dis
            if dis==1:
                return dis_min
    return dis_min
