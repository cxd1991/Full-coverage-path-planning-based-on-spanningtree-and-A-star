import json
import matplotlib.pyplot as plt
import random
import math

class Node():
    def __init__(self,position,g_cost,h_cost,parent_id):
        self.position = position
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.parent_id = parent_id

    @property
    def cost(self):
        return self.g_cost + self.h_cost

class PathWithObstacles():

    def __init__(self):
        self.blocked_x = []
        self.blocked_y = []
        self.path_x = []
        self.path_y = []
        self.way_x = []
        self.way_y = []
        self.blocked_xy = []
        self.path_xy = []
        self.new_obstables = []
        self.new_obstable_expanded = []
        self.new_obstables_set = set()
        self.new_obstable_expanded_set = set()

    # 初始化数据
    def date_read(self):
        # 读取路径数据Date,画出障碍物与路径轨迹
        file_name = "my_date.json"
        with open(file_name, 'r') as file_obj:
            Date = json.load(file_obj)
            # print("读入json文件:", Date)
        self.blocked_x = Date['blocked_x']
        self.blocked_y = Date['blocked_y']
        self.path_x = Date['path_x']
        self.path_y = Date['path_y']
        self.way_x = Date['way_x']
        self.way_y = Date['way_y']
        for x in self.way_x:
            self.path_x.append(x)
        for y in self.way_y:
            self.path_y.append(y)
        # plt.subplot(1,2,1)
        plt.plot(self.blocked_x, self.blocked_y, '.')
        # plt.plot(self.path_x, self.path_y, '.-y')
        # for i in range(0,len(path_x)-1,1):
        #     plt.plot([path_x[i], path_x[i+1]], [path_y[i],path_y[i+1]], '.-g')
        #     plt.pause(0.0001)
        # plt.show()

        # 元组格式的 障碍物 与 路径点 的坐标
        self.blocked_xy = []
        for i in range(len(self.blocked_x)):
            self.blocked_xy.append((self.blocked_x[i], self.blocked_y[i]))
        self.path_xy = []
        for i in range(len(self.path_x)):
            self.path_xy.append((self.path_x[i], self.path_y[i]))

        self.new_obstables = []
        for i in range(0, 4):
            for j in range(0, 4):
                self.new_obstables.append((250 + i, 150 + j))
        for i in range(0, 3):
            for j in range(0, 3):
                self.new_obstables.append((250 + i, 163 + j))
        for i in range(0, 3):
            for j in range(0, 3):
                self.new_obstables.append((250 + i, 140 + j))

        for i in range(len(self.new_obstables)):
            plt.plot(self.new_obstables[i][0], self.new_obstables[i][1], '.r')

        self.new_obstable_expand()
        self.new_obstables_set = set(self.new_obstables)
        self.new_obstable_expanded_set = set(self.new_obstable_expanded)
        for i in range(len(self.new_obstable_expanded)):
            plt.plot(self.new_obstable_expanded[i][0], self.new_obstable_expanded[i][1], '.b')

        # plt.show()


    # 膨胀 new_obstable
    def new_obstable_expand(self):
        print(len(self.new_obstables))
        for k in range(len(self.new_obstables)):
            for i in range(-1,-1):
                for j in range(-1,-1):
                    if (self.new_obstables[k][0]+i, self.new_obstables[k][1]+j) not in self.new_obstables:
                        self.new_obstable_expanded.append((self.new_obstables[k][0]+i, self.new_obstables[k][1]+j))


    # 根据 self.blocked_xy，self.new_obstables,self.new_obstable_expanded,self.path_xy 进行轨迹重规划
    def get_result_path(self):
        # 先对障碍物进行膨胀处理
        self.new_obstable_expand()
        self.new_obstables_set = set(self.new_obstables)
        self.new_obstable_expanded_set = set(self.new_obstable_expanded)
        result_path = []
        jump = 0
        # 依次对障碍物进行遍历，如果路径安全则添加到 result_path， 否则 重新生成一段路径接入result_path
        for i in range(len(self.path_xy)):
            # n = random.random()
            # if n>0.95:
            #     self.update_new_obstables()
            if jump!=0: # 判断需要跳过几个点
                jump = jump - 1
                continue
            if (self.path_xy[i][0],self.path_xy[i][1]) not in self.new_obstables_set and \
                    (self.path_xy[i][0],self.path_xy[i][1]) not in self.new_obstable_expanded_set :
                result_path.append((self.path_xy[i][0],self.path_xy[i][1]))
            else:
                start_point = (self.path_xy[i-1][0],self.path_xy[i-1][1])
                goal_point,m = self.get_goal_point(i)
                jump = m
                son_path_result = self.get_son_path(start_point,goal_point)
                if not son_path_result["found"]:
                    print("子路径规划失败:", son_path_result["reason"], "start=", start_point, "goal=", goal_point)
                    continue
                for point in son_path_result["path"]:
                    result_path.append(point)
        # plt.show()
        return result_path

    # 更新障碍物坐标
    def update_new_obstables(self):
        n = round(1.005*(random.random()-0.5))
        if n == 1 or n == -1:
            for i in range(len(self.new_obstables)):
                self.new_obstables[i] = (self.new_obstables[i][0],self.new_obstables[i][1] + n)
                plt.plot(self.new_obstables[i][0], self.new_obstables[i][1], '.r')


    # 使用 A* 算法 得到从start_point->goal_point的新路径
    def get_son_path(self,start_point,goal_point):

        print("寻找从",start_point,"--->",goal_point,"的路径")
        start_node = Node(start_point,0,0,(-1,-1))
        goal_node = Node(goal_point,0,0,(-1,-1))
        open_set = {}
        close_set = {}
        open_set[start_node.position] = start_node
        motion = [ [1,0,1], [-1,0,1], [0,1,1], [0,-1,1] ]
        obstacles = self.new_obstables_set | self.new_obstable_expanded_set

        if start_point in obstacles or goal_point in obstacles:
            print("[A*] 起点或终点位于障碍物中: start=", start_point, "goal=", goal_point)
            return {"found": False, "path": [], "reason": "start_or_goal_in_obstacle"}

        x_values = [p[0] for p in self.path_xy] + [start_point[0], goal_point[0]]
        y_values = [p[1] for p in self.path_xy] + [start_point[1], goal_point[1]]
        bounds = (min(x_values), max(x_values), min(y_values), max(y_values))

        def is_in_bounds(position):
            x, y = position
            min_x, max_x, min_y, max_y = bounds
            return min_x <= x <= max_x and min_y <= y <= max_y

        while 1:
            if not open_set:
                return {"found": False, "path": [], "reason": "open_set_exhausted"}
            c_id = min(open_set, key = lambda o : open_set[o].cost )
            current = open_set[c_id]
            if current.position == goal_node.position:
                print("找到路径")
                goal_node.parent_id = current.parent_id
                goal_node.g_cost = current.g_cost
                goal_node.h_cost = current.h_cost
                path = self.find_final_path(goal_node,close_set)
                return {"found": True, "path": path, "reason": "ok"}

            del open_set[c_id]
            close_set[c_id] = current

            for move_x,move_y,move_cost in motion:
                next_pos = (current.position[0]+move_x,current.position[1]+move_y)
                if not is_in_bounds(next_pos):
                    continue
                node = Node(next_pos,\
                            current.g_cost+move_cost,\
                            ((next_pos[0] - goal_node.position[0]) ** 2 + (next_pos[1] - goal_node.position[1]) ** 2) ** 0.5,\
                            (current.position[0],current.position[1]))
                n_id = node.position
                if n_id in close_set:
                    continue
                if n_id in obstacles:
                    continue
                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id].g_cost = node.g_cost
                        open_set[n_id].h_cost = node.h_cost
                        open_set[n_id].parent_id = node.parent_id

    # 回溯路径
    def find_final_path(self,goal,close):
        path = []
        path.append(goal.position)
        p_id = goal.parent_id
        while p_id != (-1,-1):
            n = close[p_id]
            path.append(n.position)
            p_id = n.parent_id
        path.reverse()
        del path[0]
        print(path)
        return path

    # 得到在第i个位置处存在障碍物时往后遍历得到合格的目标点，便于下一步的路径重规划
    def get_goal_point(self,k):
        jump = 0
        for i in range(k+1,len(self.path_xy)+1):
            # 记录跳过的点数
            jump = jump + 1
            if (self.path_xy[i][0],self.path_xy[i][1]) not in self.new_obstables_set and \
                    (self.path_xy[i][0],self.path_xy[i][1]) not in self.new_obstable_expanded_set :
                return (self.path_xy[i][0],self.path_xy[i][1]), jump
            else:
                continue


path_obj = PathWithObstacles()
path_obj.date_read()
result = path_obj.get_result_path()


for i in range(0,len(result)-1,1):
    plt.plot([result[i][0],result[i+1][0]],[result[i][1],result[i+1][1]],'.-y')
    # plt.pause(.0001)
plt.show()
