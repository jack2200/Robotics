import random
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
import time

GAMMA = 50.0 #Planning constant used in finding near nodes.
D = 3 #State space dimension is 2 in this problem.
EPSILON = 0.01
class Node():
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        self.cost = 0.0
        self.parent = None
def sign(x):
	return 1-(x<=0)
class RRTStar3D():
    
    def __init__(self,start,goal,obstacle_list,search_area,
                 delta_q = 0.4, goal_sample_rate = 10, maxIter = 1000,buffer=0):
        self.start = Node(start[0],start[1],start[2])
        self.goal = Node(goal[0],goal[1],goal[2])
        self.search_area = search_area
        self.obstacle_list = obstacle_list
        self.delta_q = delta_q
        self.goal_sample_rate = goal_sample_rate
        self.maxIter = maxIter
        self.buffer = buffer
        
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
                pass
        # generate path
        ind = self.get_best_last_index()
        if ind is None:
            return None
        else:
            path = self.gen_path(ind)
            return path
    def draw_graph(self,ax):
        for node in self.nodeList:
            if node.parent is not None:
                ax.plot([node.x, self.nodeList[node.parent].x], [node.y, self.nodeList[node.parent].y],zs=[node.z, self.nodeList[node.parent].z],c='blue')
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        for o in self.obstacle_list:
            x = o[0] + o[3]*np.cos(u)*np.sin(v)
            y = o[1] + o[3]*np.sin(u)*np.sin(v)
            z = o[2] + o[3]*np.cos(v)
            ax.plot_surface(x, y, z, color="r")
        ax.scatter(self.start.x, self.start.y,self.start.z, "xr",s=100)
        ax.scatter(self.goal.x, self.goal.y,self.goal.z, "xr",s=100)
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
        path = [[self.goal.x, self.goal.y, self.goal.z]]
        while self.nodeList[goal_ind].parent is not None:
            node = self.nodeList[goal_ind]
            path.append([node.x,node.y,node.x])
            goal_ind = node.parent
        path.append([self.start.x,self.start.y, self.start.z])
        return path
    def sample(self):
        p = random.randint(0,100)
        if p > self.goal_sample_rate:
            rnd = [random.randint(self.search_area[0],self.search_area[1]),
                       random.randint(self.search_area[0],self.search_area[1]),
		       random.randint(self.search_area[0],self.search_area[1])]
        else:
            rnd = [self.goal.x,self.goal.y,self.goal.z]
        return rnd
    
    def nearest(self,random_point):
        dlist = [(node.x - random_point[0])**2 + (node.y - random_point[1]) **2 + (node.z - random_point[2]) **2  for node in self.nodeList]
        return self.nodeList[dlist.index(min(dlist))]
        
    def steer(self,rnd,nearest):
        newNode = Node(rnd[0],rnd[1],rnd[2])
        dist = math.sqrt((newNode.x - nearest.x)**2 + (newNode.y - nearest.y)**2 + (newNode.z - nearest.z)**2)
        if dist <= self.delta_q:
            pass
        else: # maximum possible extend length
            newNode.x = nearest.x + self.delta_q * (newNode.x - nearest.x)/dist
            newNode.y = nearest.y + self.delta_q * (newNode.y - nearest.y)/dist
            newNode.z = nearest.z + self.delta_q * (newNode.z - nearest.z)/dist
            
        newNode.cost = float("inf")
        newNode.parent = None
        return newNode
    
    def near(self,node):
        m = len(self.nodeList)
        k = GAMMA*math.sqrt((math.log(m)/m))
        dlist = [(n.x - node.x)**2 + (n.y - node.y)**2 + (n.z - node.z)**2 for n in self.nodeList]
        near_node_indices = [dlist.index(dist) for dist in dlist if dist <= k**2]
        return near_node_indices
    
    # This collision model only considers circles.
    def obstacle_free(self,node):
        for i in range(0,len(self.obstacle_list)):
            cx = self.obstacle_list[i][0]
            cy = self.obstacle_list[i][1]
            cz = self.obstacle_list[i][2]
            r = self.obstacle_list[i][3]
            if (node.x - cx)**2 + (node.y - cy)**2 + (node.z - cz)**2 <= r**2 + self.buffer**2 :
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
            dz = z_new.z - self.nodeList[i].z
            d = math.sqrt(dx**2 + dy**2 + dz**2)
            stridex = dx/10.0
            stridey = dy/10.0
            stridez = dz/10.0
            tmpx,tmpy,tmpz = self.nodeList[i].x,self.nodeList[i].y,self.nodeList[i].z
            not_collision = True
            while abs((tmpx - z_new.x)*sign(stridex))>1e-2:
                while abs((tmpy - z_new.y)*sign(stridey))>1e-2:
                    while abs((tmpz - z_new.z)*sign(stridez))>1e-2:
                        not_collision = self.check_intermediate_collision(Node(tmpx,tmpy,tmpz))
                        tmpz += stridez
                        if not_collision == False:
                            break
                    tmpy += stridey
                    if not_collision == False:
                        break          
                tmpx += stridex
                if not_collision == False:
                    break
            if not_collision:
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

    def check_intermediate_collision(self, nearNode):
        if not self.obstacle_free(nearNode):
                return False
        return True

    def rewire(self, newNode, nearinds):
        nnode = len(self.nodeList)
        for i in nearinds:
            nearNode = self.nodeList[i]

            dx = newNode.x - nearNode.x
            dy = newNode.y - nearNode.y
            dz = newNode.z - nearNode.z
            d = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

            scost = newNode.cost + d

            if nearNode.cost > scost:
		    #print "Rewiring ",nearNode.x,",",nearNode.y, "with ",newNode.x,",",newNode.y
                stridex = dx/10.0
                stridey = dy/10.0
                stridez = dz/10.0
                tmpx,tmpy,tmpz = self.nodeList[i].x,self.nodeList[i].y,self.nodeList[i].z
                not_collision = True
                
                while abs((tmpx - newNode.x)*sign(stridex))>1e-2:
                    while abs((tmpy - newNode.y)*sign(stridey))>1e-2:
                        while abs((tmpz - newNode.z)*sign(stridez))>1e-2:
                            not_collision = self.check_intermediate_collision(Node(tmpx,tmpy,tmpz))
                            if not_collision == False:
                                break
                            tmpz += stridez
                        if not_collision == False:
                            break
                        tmpy += stridey
                    if not_collision == False:
                        break
                    tmpx += stridex
                if not_collision:
                    nearNode.parent = nnode - 1
                    nearNode.cost = scost


def main():
    obstacle_list = [
        [15, 15, 17,5],
        [30, 46, 21,8],
        [35, 63, 25,2],
        [31, 50, 36,4.5],
        [45, 48, 54,6],
        [67, 50, 75, 5]
    ]

    rrt = RRTStar3D([0,0,0],[50,67,73.5],obstacle_list,[-2,100],maxIter=2500,buffer = 5.0)
    start = time.clock()
    path = rrt.plan()
    end = time.clock()
    if path is None:
        print("Cannot find a suitable path")
    else:
        print(path)
        fig = plt.figure()
        ax = Axes3D(fig)
        rrt.draw_graph(ax)
        
        for i in range(0,len(path)-1):
            ax.plot([path[i][0], path[i+1][0]], [path[i][1], path[i+1][1]],zs=[path[i][2], path[i+1][2]],c='green')
        
        plt.show()
        
    
    print("%.7g s"%(end-start)) 
if __name__=='__main__':
    main()
