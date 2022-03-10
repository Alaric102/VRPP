import numpy as np

class Algorithm:
    def __init__(self, env_map, conf_space, start_point, goal_point):
        self.env_map = env_map
        self.conf_space = conf_space
        self.start_point = start_point
        self.goal_point = goal_point
        self.path = []

        # map which hold visit status {0, 1} and cost of visiting
        self.visit_map = np.zeros_like(conf_space, dtype=tuple)
        self.visit_map.fill((0,0))
        self.visited_num = 0
        self.queue = []
        self.parent_table = []

    def wrap_angle(self, angle):
        while angle > 3:
            angle -= 3
        while angle < 0:
            angle += 3
        return angle

    def isEqual(self, lft, rht): 
        return (lft == rht).all()

    def getActionSpace(self):
        action_space = []
        for dof in range(3):
            for step in [1, -1]:
                action = np.zeros( (3,), dtype=int )
                action[dof] = step
                action_space.append(action)
        return action_space

    def isVisited(self, point): 
        item = self.visit_map[point[0], point[1], point[2]]
        return ( item[0] == 1)

    def setVisited(self, point, cost):
        item = (1, cost)
        self.visit_map[point[0], point[1], point[2]] = item
        self.visited_num += 1

    def getVisited(self, point):
        item = self.visit_map[point[0], point[1], point[2]]
        return item

    def queueInsert(self, point, cost):
        for i in range(len(self.queue)):
            item = self.queue[i]
            if item[1] <= cost:
                self.queue.insert(i, (point, cost) )
                return
        self.queue.append( (point, cost) )

    def isObstacle(self, point):
        return ( self.conf_space[point[0], point[1], point[2]] == 1)

    def queueGet(self):
        return self.queue.pop(-1)

    def getParentPoint(self, point):
        for i in self.parent_table:
            if (i[1] == point).all():
                return i[0]
        return np.empty

    def getBranch(self):
        child_point = self.goal_point
        path = [child_point]
        while  not (child_point == self.start_point).all():
            parent_point = self.getParentPoint(child_point)
            if (parent_point == np.empty).all():
                return path
            path.append(parent_point)
            child_point = parent_point
        return path

    def getPath(self):
        path = self.getBranch()
        path.reverse()
        return path

    def getHeuristic(self, point):
        return 0