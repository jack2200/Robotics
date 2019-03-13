import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import time

GAMMA = 50.0 #Planning constant used in finding near nodes.
D = 2 #State space dimension is 2 in this problem. 
class Node():
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None

class RRT():
    
    def __init__(self,start,goal,obstacle_list,search_area,
                 delta_q = 0.4, goal_sample_rate = 10, maxIter = 1000):
        self.start = Node(start[0],start[1])
        self.goal = Node(goal[0],goal[1])
        self.search_area = search_area
        self.obstacle_list = obstacle_list
        self.delta_q = delta_q
        self.goal_sample_rate = goal_sample_rate
        self.maxIter = maxIter
        
    def plan(self):
        self.nodeList = [self.start]
        for i in range(self.maxIter):
            # Exploration probability.
            
            z_rnd = self.sample()
            z_nearest = self.nearest(z_rnd)
            z_new = self.steer(z_rnd,z_nearest)
            if self.obstacle_free(z_new):
                z_near_inds = self.near(z_new) # A list of node indices
                z_new = self.choose_parent(z_near_inds,z_new)
                self.nodeList.append(z_new)
                self.rewire(z_new,z_near_inds)
            if i%5==0:
                pass#self.draw_graph(z_rnd)
        # generate path
        ind = self.get_best_last_index()
        if ind is None:
            return None
        else:
            path = self.gen_path(ind)
            return path
    def draw_graph(self,z_rnd=None):
        plt.clf()
        if z_rnd is not None:
            plt.plot(z_rnd[0], z_rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [node.y, self.nodeList[node.parent].y], "-g")
            plt.plot(node.x,node.y,"bo",ms=2)
        for o in self.obstacle_list:
            obs = plt.Circle((o[0], o[1]), o[2], color='black')
            plt.gcf().gca().add_artist(obs)
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis([-2, 30, -2, 30])
        plt.grid(True)
        plt.pause(0.0001)
    def get_best_last_index(self):
        dist_to_goal_list = [np.linalg.norm([node.x - self.goal.x,node.y - self.goal.y]) for node in self.nodeList]
        valid_indices = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i<=self.delta_q]
        
        if len(valid_indices)==0:
            return None
        mincost = min([self.nodeList[i].cost for i in valid_indices])
        for i in valid_indices:
            if self.nodeList[i].cost == mincost:
                return i

        return None
    
    def gen_path(self,goal_ind):
        path = [[self.goal.x, self.goal.y]]
        while self.nodeList[goal_ind].parent is not None:
            node = self.nodeList[goal_ind]
            path.append([node.x,node.y])
            goal_ind = node.parent
        path.append([self.start.x,self.start.y])
        return path
    def sample(self):
        p = random.randint(0,100)
        if p > self.goal_sample_rate:
            rnd = [random.randint(self.search_area[0],self.search_area[1]),
                       random.randint(self.search_area[0],self.search_area[1])]
        else:
            rnd = [self.goal.x,self.goal.y]
        return rnd
    
    def nearest(self,random_point):
        dlist = [(node.x - random_point[0])**2 + (node.y - random_point[1]) **2 for node in self.nodeList]
        return self.nodeList[dlist.index(min(dlist))]
        
    def steer(self,rnd,nearest):
        theta = math.atan2(rnd[1] - nearest.y,rnd[0] - nearest.x)
        newNode = Node(rnd[0],rnd[1])
        dist = math.sqrt((newNode.x - nearest.x)**2 + (newNode.y - nearest.y)**2)
        if dist <= self.delta_q:
            pass
        else: # maximum possible extend length
            newNode.x = nearest.x + self.delta_q * math.cos(theta)
            newNode.y = nearest.y + self.delta_q * math.sin(theta)
        
        newNode.cost = float("inf")
        newNode.parent = None
        return newNode
    
    def near(self,node):
        m = len(self.nodeList)
        k = GAMMA*math.sqrt((math.log(m)/m))
        dlist = [(n.x - node.x)**2 + (n.y - node.y) **2 for n in self.nodeList]
        near_node_indices = [dlist.index(dist) for dist in dlist if dist <= k**2]
        return near_node_indices
    
    # This collision model only considers circles.
    def obstacle_free(self,node):
        for i in range(0,len(self.obstacle_list)):
            cx = self.obstacle_list[i][0]
            cy = self.obstacle_list[i][1]
            r = self.obstacle_list[i][2]
            if (node.x - cx)**2 + (node.y - cy)**2 <= r**2:
                return False
        return True
    
    # Chooses the best parent from nearby nodes.
    def choose_parent(self,z_near_inds,z_new):
        if len(z_near_inds) == 0:
            return z_new
        dlist = []
        for i in z_near_inds:
            dx = z_new.x - self.nodeList[i].x
            dy = z_new.y - self.nodeList[i].y
            d = math.sqrt(dx**2 + dy**2)
            theta = math.atan2(dy,dx)
            if self.check_intermediate_collision(self.nodeList[i],d,theta):
                dlist.append(self.nodeList[i].cost + d)
            else:
                dlist.append(float("inf"))
        
        cost = min(dlist)
        if cost==float("inf"):
            print("Minimum cost is infinite. Namely, there isn't any collision-free nodes as a parent candidate")
            return z_new
        mini = z_near_inds[dlist.index(cost)]
        z_new.cost = cost
        z_new.parent = mini
        
        return z_new
    def check_intermediate_collision(self, nearNode, d, theta):

        tmpNode = copy.deepcopy(nearNode)
        
        for i in range(int(d / self.delta_q)):
            tmpNode.x += self.delta_q * math.cos(theta)
            tmpNode.y += self.delta_q * math.sin(theta)
            if not self.obstacle_free(tmpNode):
                return False
        return True
    def rewire(self, newNode, nearinds):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            d = math.sqrt(dx ** 2 + dy ** 2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
		#print "Rewiring ",nearNode.x,",",nearNode.y, "with ",newNode.x,",",newNode.y
                theta = math.atan2(dy, dx)
                if self.check_intermediate_collision(nearNode, d, theta):
                    
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost


def main():
    obstacle_list = [
        [5, 5, 1],
        [3, 6, 2],
        [3, 8, 2],
        [3, 10, 2],
        [7, 5, 2],
        [9, 5, 2]
    ]

    rrt = RRT([0,0],[20,20],obstacle_list,[-2,25],maxIter=500)
    start = time.clock()
    path = rrt.plan()
    end = time.clock()
    if path is None:
        print("Cannot find a suitable path")
    else:
        rrt.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.show()
        print(path)
    
    print("%.7g s"%(end-start)) 
if __name__=='__main__':
    main()
