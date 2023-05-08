#!/usr/bin/env python3

from random import random
from z3 import *
import numpy as np
import time
from math import *
from statistics import *
from random_lib import *
from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
import sys
sys.setrecursionlimit(10**6)
clusters = []
class Cluster:
    def __init__(self,m,n):
        # Get the dimensions of the grid
        self.rows = m
        self.cols = n
        
        # Create a visited array to keep track of which cells have been visited
        self.nv = len(visible_cells)

        global clusters
        clusters = []
        self.cell2cluster = dict()
        self.cells_per_cluster = []
    

    def traverse(self,r, c):
        # Check if the current cell is out of bounds or has already been visited
        if r < 0 or r >= self.rows or c < 0 or c >= self.cols or visited_map[r][c]:
            return
        
        # Check if the current cell is a 0
        if map[r][c] != 0.5:
            return
        
        # Mark the current cell as visited
        visited_map[r][c] = True
        self.component.append((c,r))
        (x,y) = (c,r)
        key = str(x)+'+'+str(y)
        self.cell2cluster[key] = len(clusters)
        
        # Recursively traverse the neighbors of the current cell
        self.traverse(r + 1, c)  # right
        self.traverse(r - 1, c)  # left
        self.traverse(r, c + 1)  # down
        self.traverse(r, c - 1)  # up
    
    def make_clusters(self):
        for  (x,y) in visible_cells:
            (r,c) = (y,x)
            # Skip cells that have already been visited
            if visited_map[r][c]:
                continue
            # Initialize a new connected component as a list of coordinates
            self.component = []
            # Traverse the connected component and add the coordinates of each cell to the list
            self.traverse(r, c)
            # Add the connected component to the list of components
            clusters.append(np.array(self.component))
            self.cells_per_cluster.append(len(self.component))
        for (x,y) in visible_cells:
            r = y
            c = x
            visited_map[r][c] = False

        return np.array(self.cells_per_cluster),self.cell2cluster

class SMT:
    def __init__(self,R,T,Dx,Dy,vis_dist,n_neighbors):
        self.s = Optimize()
        self.s.set(priority='lazy',  incremental=True, timeout=10000) # iteration_limit=10000 , priority : lazy 
        self.R = R
        self.T = T
        self.Dx = Dx
        self.Dy = Dy
        self.m = len(map[0])
        self.n = len(map)
        self.vis_dist = vis_dist
        self.n_neighbors = n_neighbors
        self.W_Cost  = 1000 # 100
        self.W_Reward = 1000 # 10
        self.WR_Cov = max(Dx,Dy)*(10+0.1) # 300
        self.WR_Vis = max(Dx,Dy)*(10+0.1) # 300
        self.reachable_cells = [[] for i in range(R)]
        self.X = [[Int("x%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
        self.Y = [[Int("y%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
        #self.P = [[Int("p_%s_%s" % (i, j)) for j in range(T-1)] for i in range(R)]
        self.C = [[Real("c_%s_%s" % (i, j)) for j in range(T-1)] for i in range(R)]
        self.Re = [[Real("re_%s_%s" % (i, j)) for j in range(T-1)] for i in range(R)]
        self.total_cost = Real('total_cost')
        self.total_reward = Real('total_reward')
        self.Obj_func = Real('Obj_func')
        self.constraints = []

    def assign_Bots2Clusters(self,bot_loc):
        self.clusters_assigned = []
        self.bots_assigned = []
        nc = len(clusters)
        nr = self.R
        D = np.zeros((nc,nr))
        self.D_min_idx = np.zeros((nc,nr))
        count_clusters_unassigned = nc
        count_bots_unassigned = nr

        for c in range(nc):
            
            self.clusters_assigned.append([])

            for r in range(nr):
                if c == 0:
                    self.bots_assigned.append([])

                (rx,ry) = bot_loc[r]
                dist = abs(rx-clusters[c][:,0]) + abs(ry-clusters[c][:,1])
                idxs = dist.argsort()[:self.n_neighbors]
                
                d_min  = dist[idxs[0]]
                #d_avg = np.average(dist[idxs])
                self.D_min_idx[c][r] = idxs[0]
                D[c][r] = d_min

                if  d_min == 1:
                    if len(self.bots_assigned[r]) == 0 :
                        self.clusters_assigned[c].append(r)
                        self.bots_assigned[r].append(c)
                        count_bots_unassigned += -1
                    # else:
                    #     c_prev = self.bots_assigned[r][0]
                    #     if len(clusters[c])<len(clusters[c_prev]):
                    #         self.clusters_assigned[c_prev].remove(r)
                    #         self.clusters_assigned[c].append(r)
                    #         self.bots_assigned[r][0] = c
            
            if len(self.clusters_assigned[c]) > 0:
                count_clusters_unassigned += -1


        Idx_sorted = D.argsort(axis=None)

        for idx in Idx_sorted:
            c = idx // nr
            r = idx % nr
            
            if len(self.clusters_assigned[c]) == 0 and len(self.bots_assigned[r]) == 0 and self.inbetween_vis(r,c)==0:
                self.clusters_assigned[c].append(r)
                self.bots_assigned[r].append(c)
                count_bots_unassigned += -1
                count_clusters_unassigned += -1
        
        self.bots_per_cluster = np.array([ len(bots) if len(bots)>0 else 1000000 for bots in self.clusters_assigned])
        cells_per_bot = self.cells_per_cluster / self.bots_per_cluster
        
        bots_unassigned = []
        for r in range(self.R) :
            if len(self.bots_assigned[r]) == 0:
                bots_unassigned.append(r)

        while (count_bots_unassigned != 0):
            c = np.argmax(cells_per_bot)
            dist = D[c][bots_unassigned]
            idxs = dist.argsort()

            assigned = False
            for i in idxs :
                r = bots_unassigned[i]  
                if (self.inbetween_vis(r,c)==0):
                    bots_unassigned.pop(i)
                    self.clusters_assigned[c].append(r)
                    self.bots_assigned[r].append(c)
                    self.bots_per_cluster[c]+=1
                    cells_per_bot[c] = self.cells_per_cluster[c]/self.bots_per_cluster[c]
                    count_bots_unassigned += -1
                    assigned = True
                    break
            if(assigned == False):
                self.bots_per_cluster[c] = 1000000
                cells_per_bot[c] = self.cells_per_cluster[c]/self.bots_per_cluster[c]
    
    def new_assign_Bots2Clusters(self,bot_loc):
        self.clusters_assigned = []
        self.bots_assigned = []
        nc = len(clusters)
        nr = self.R
        D = np.zeros((nc,nr))
        self.D_min_idx = np.zeros((nc,nr))
        count_clusters_unassigned = nc
        count_bots_unassigned = nr
        for r in range(nr):
            self.bots_assigned.append([])
        for c in range(nc):
            self.clusters_assigned.append([])
        
        for r in range(nr):
            c_near = []
            (rx,ry) = bot_loc[r]
            for c in range(nc):
                dist = abs(rx-clusters[c][:,0]) + abs(ry-clusters[c][:,1])
                idxs = dist.argsort()[:self.n_neighbors]
                d_min  = dist[idxs[0]]
                #d_avg = np.average(dist[idxs])
                self.D_min_idx[c][r] = idxs[0]
                D[c][r] = d_min
                if d_min==1:
                    c_near.append(c)
            
            if len(c_near)>0:
                c_near_cells = np.array(self.cells_per_cluster[c_near])
                idxs = c_near_cells.argsort()
                c_near_min = c_near[idxs[0]]
                c_near_max = c_near[idxs[len(idxs)-1]]
                c = 0
                if len(self.clusters_assigned[c_near_min]) == 0:
                    c = c_near_min
                else:
                    c = c_near_max
                self.clusters_assigned[c].append(r)
                self.bots_assigned[r].append(c)
                count_bots_unassigned += -1
                count_clusters_unassigned += -1

        Idx_sorted = D.argsort(axis=None)

        for idx in Idx_sorted:
            c = idx // nr
            r = idx % nr
            
            if len(self.clusters_assigned[c]) == 0 and len(self.bots_assigned[r]) == 0 and self.inbetween_vis(r,c)==0:
                self.clusters_assigned[c].append(r)
                self.bots_assigned[r].append(c)
                count_bots_unassigned += -1
                count_clusters_unassigned += -1
        
        
        
        self.bots_per_cluster = np.array([ len(bots) if len(bots)>0 else 1000000 for bots in self.clusters_assigned])
        cells_per_bot = self.cells_per_cluster / self.bots_per_cluster
        
        bots_unassigned = []
        for r in range(self.R) :
            if len(self.bots_assigned[r]) == 0:
                bots_unassigned.append(r)

        while (count_bots_unassigned != 0):
            c = np.argmax(cells_per_bot)
            dist = D[c][bots_unassigned]
            idxs = dist.argsort()

            assigned = False
            for i in idxs :
                r = bots_unassigned[i]  
                if (self.inbetween_vis(r,c)==0):
                    bots_unassigned.pop(i)
                    self.clusters_assigned[c].append(r)
                    self.bots_assigned[r].append(c)
                    self.bots_per_cluster[c]+=1
                    cells_per_bot[c] = self.cells_per_cluster[c]/self.bots_per_cluster[c]
                    count_bots_unassigned += -1
                    assigned = True
                    break
            if(assigned == False):
                self.bots_per_cluster[c] = 1000000
                cells_per_bot[c] = self.cells_per_cluster[c]/self.bots_per_cluster[c]

    def inbetween_vis(self,r,c):
        vis_array = copy.copy(visible_cells)
        vis_array = np.array(vis_array)
        (rx,ry) = self.bot_loc[r]
        (vx,vy) = clusters[c][int(self.D_min_idx[c][r])]
        dx = rx-vx
        dy = ry-vy
        dx_array = rx-vis_array[:,0]
        dy_array = ry-vis_array[:,1]
        filtered_idxs = (dx*dx_array >= 0) & (dy*dy_array >= 0) & (abs(dx_array)+abs(dy_array) < abs(dx)+abs(dy))
        if(dx==0):
            filtered_idxs = filtered_idxs & (dx_array==0)
        elif(dy==0):
            filtered_idxs = filtered_idxs & (dy_array==0)

        vis_array = vis_array[filtered_idxs]
        return len(vis_array)

    def make_and_assign_clusters(self,bot_loc):
        self.bot_loc = bot_loc
        self.cells_per_cluster, self.cell2cluster = Cluster(self.m,self.n).make_clusters()
        self.new_assign_Bots2Clusters(bot_loc)

    def init_bot_loc(self,bot_loc):
        self.bot_loc = bot_loc
        
        for r in range (self.R):
            (x,y) = bot_loc[r]
            self.constraints.append(And (self.X[r][0] == int (x),   self.Y[r][0] == int (y)))              # Assign the initial x and y coordinates 
            self.collect_reachables(r,self.vis_dist)                                # collect other reachable locations available  

    ##########       
    def collect_reachables(self,r,d):
        (rx,ry) = self.bot_loc[r]
       
        xl = int (max (0, rx-d))
        xh = int (min (self.Dx, rx+d+1))
        yl = int (max (0, ry-d))
        yh = int (min (self.Dy, ry+d+1))

        for x in range (xl, xh):
            for y in range (yl, yh):
                # Collect all reachable cells from visible cells according to path length T (reachability)
                if ((map[y][x] == 0.5 or map[y][x] == 1.0) and (abs (x - rx) + abs (y - ry) <= d) ) :  

                    self.reachable_cells[r].append((x,y))
                

    def motion_primitive(self):
        for r in range (self.R):
            for t in range (self.T-1):
                self.constraints.append (And (self.P[r][t] <= 4, self.P[r][t] >= 0))  # Only 5 motion primitives are allowed
                self.constraints.append (Or (self.C[r][t] == 1 , self.C[r][t] == 3 )) # Only 2 cost values are  allowed 

                # For robot r at time t  , If we choose an allowed value of P then the corresponding cost and next state allowed is defined
                self.constraints.append(Implies(self.P[r][t] == 0, And(self.X[r][t+1] == self.X[r][t],  self.Y[r][t+1] == self.Y[r][t],  self.C[r][t] == 3))) # same
                self.constraints.append(Implies(self.P[r][t] == 1, And(self.X[r][t+1] == self.X[r][t]+1,self.Y[r][t+1] == self.Y[r][t],  self.C[r][t] == 1))) # right
                self.constraints.append(Implies(self.P[r][t] == 2, And(self.X[r][t+1] == self.X[r][t]-1,self.Y[r][t+1] == self.Y[r][t],  self.C[r][t] == 1))) # left
                self.constraints.append(Implies(self.P[r][t] == 3, And(self.X[r][t+1] == self.X[r][t],  self.Y[r][t+1] == self.Y[r][t]+1,self.C[r][t] == 1))) # up
                self.constraints.append(Implies(self.P[r][t] == 4, And(self.X[r][t+1] == self.X[r][t],  self.Y[r][t+1] == self.Y[r][t]-1,self.C[r][t] == 1))) # down             
    
    def action_cost(self,current_loc,next_loc):
        same_cell_cost = 3
        different_cell_cost = 1
        if current_loc == next_loc:
            return same_cell_cost
        else:
            return different_cell_cost
    ##########
    def reward(self,r,current_loc,next_loc):
        (nx,ny) = (int(next_loc[0]),int(next_loc[1]))
        if(map[ny][nx] == 0.5):
            key = str(nx)+'+'+str(ny)
            c = self.cell2cluster[key]
            cov = self.surroundings(next_loc,1)
           # return (cov + 1/len(clusters[c]))*self.WR_Vis #cell_age[ny][nx]+
            if(self.bots_assigned[r][0]==c):
                return (cov + 1/len(clusters[c]))*self.WR_Vis #cell_age[ny][nx]+
            else:
                return -1000
        elif(map[ny][nx] == 1.0):
            return self.near(r,current_loc, next_loc)*self.WR_Cov
        else:
            return -1000
    
    def near(self,r,current_loc,next_loc):
        (nx,ny) = (next_loc[0],next_loc[1])
        (rx,ry) = (current_loc[0],current_loc[1]) 
        # if(len(self.visible_cells)==0):
        #     return 0
        # visible_cells = np.array (self.visible_cells)
        if(len(clusters[self.bots_assigned[r][0]])==0):
            return 0
        np_visible_cells = np.array (clusters[self.bots_assigned[r][0]])
        dist = abs (np_visible_cells[:,0] - rx) + abs (np_visible_cells[:,1] - ry)
        idxs = dist.argsort ()[:self.n_neighbors]
        safe_visible_cells = np_visible_cells[idxs]
        k = len(safe_visible_cells)
        total_d = 0
        for loc in safe_visible_cells:
           d = abs (loc[0] - nx) + abs (loc[1] - ny)
           total_d += d 
        return k/total_d 
    
    ##########
    # def near(self,r,current_loc,next_loc):
    #     (nx,ny) = (next_loc[0],next_loc[1])
    #     (rx,ry) = (current_loc[0],current_loc[1]) 

    #     total_w_d = 0
    #     key = str(rx)+'+'+str(ry)
    #     k = len(self.nearest_vis_cells.get(key))
    #     Total_W = 0
    #     for loc in self.nearest_vis_cells[key]: 
    #        d = abs (loc[0] - nx) + abs (loc[1] - ny)

    #        (x,y) = (loc[0],loc[1])
    #        key = str(x)+'+'+str(y)
           
    #        common_ratio = self.R/self.vis_common.get(key)
    #        w = pow(common_ratio,3)
    #        Total_W += w
    #        total_w_d += d*w
    #     return Total_W/total_w_d 
    
    ##########
    def Visible_cells_common_count(self):
        self.vis_common = dict({})
        self.nearest_vis_cells = dict({})
       
        for r in range(self.R):
            (rx,ry) = self.bot_loc[r]
            key = str(rx)+'+'+str(ry)
            self.nearest_vis_cells[key] = []
           
            if(len(clusters[self.bots_assigned[r][0]])==0):
                return 0

            visible_cells = np.array (clusters[self.bots_assigned[r][0]])
            dist = abs (visible_cells[:,0] - rx) + abs (visible_cells[:,1] - ry)
            idxs = dist.argsort ()[:self.n_neighbors]
            self.nearest_vis_cells[key] = visible_cells[idxs]
            for cell in self.nearest_vis_cells[key]:
                (x,y) = (cell[0],cell[1])
                key = str(x)+'+'+str(y)
                if self.vis_common.get(key) == None :
                    self.vis_common[key] = 1
                else:
                    self.vis_common[key] += 1

    def collision_avoidance(self):
        
        for t in range(self.T-1):
            for r1 in range (self.R):
                for r2 in range (r1+1,self.R):
                    
                    # Both x and y coordinates of r1 & r2 at time t+1 cannot be equal
                    self.constraints.append (Not( And(self.X[r1][t+1] == self.X[r2][t+1], self.Y[r1][t+1] == self.Y[r2][t+1])))
                    # Head on collision or Swaping position collision
                    self.constraints.append(Not (And(And(self.X[r1][t+1] == self.X[r2][t],self.Y[r1][t+1] == self.Y[r2][t]),And(self.X[r2][t+1] == self.X[r1][t],self.Y[r2][t+1] == self.Y[r1][t]))))
        

    def d_bots(self,r1,r2):
        (x1,y1) = self.bot_loc[r1]
        (x2,y2) = self.bot_loc[r2]
        return abs(x1-x2)+abs(y1-y2)
    
    def new_collision_avoidance(self):
        for t in range(self.T-1):
            for r1 in range (self.R):
                for r2 in range(r1+1,self.R):
                    # Both x and y coordinates of r1 & r2 at time t cannot be equal
                    if(self.d_bots(r1,r2)<=2):
                        
                        self.constraints.append (Not( And(self.X[r1][t+1] == self.X[r2][t+1], self.Y[r1][t+1] == self.Y[r2][t+1])))
                            
                        # Head on collision or Swaping position collision
                        if(self.d_bots(r1,r2)==1):
                            self.constraints.append(Not (And(And(self.X[r1][t+1] == self.X[r2][t],self.Y[r1][t+1] == self.Y[r2][t]),And(self.X[r2][t+1] == self.X[r1][t],self.Y[r2][t+1] == self.Y[r1][t]))))    
        
    def obstacle_avoidance(self,obst_loc):
        self.obst_loc = obst_loc
        for r in range (self.R):
            for t in range (1,self.T):
                for obst in obst_loc:
                    # Both the x & y coordinates of r at time t cannot be equal to that of obstacle coordinates
                    self.constraints.append (Not( And (self.X[r][t] == obst[0], self.Y[r][t] == obst[1])))
                # stay within  the grid bounds
                self.constraints.append (And (self.X[r][t] < self.Dx, self.X[r][t] >= 0))
                self.constraints.append (And (self.Y[r][t] < self.Dy, self.Y[r][t] >= 0))

    def visit_reachable_cells(self):
        #self.Visible_cells_common_count()
        for r in range (self.R):
            for t in range (self.T-1):  
                # A robot r at time t must choose  a cell from all the reachable cells
                self.constraints.append (Or ([And (self.X[r][t+1] == x, self.Y[r][t+1] == y) for (x,y) in self.reachable_cells[r]])) 
                curr = self.bot_loc[r]
                for next in self.reachable_cells[r]:
                    cx,cy = curr
                    nx,ny = next
                    self.constraints.append(Implies(And(And (self.X[r][t] == int(cx), self.Y[r][t] == int(cy)),And (self.X[r][t+1] == int(nx), self.Y[r][t+1] == int(ny))),And(self.Re[r][t] == self.reward(r,[cx,cy],[nx,ny]),self.C[r][t] == self.action_cost([cx,cy],[nx,ny]))))
                    #self.constraints.append(Implies(Or(Not(And (self.X[r][t] == int(cx), self.Y[r][t] == int(cy))),Not(And (self.X[r][t+1] == int(nx), self.Y[r][t+1] == int(ny)))),self.Re[r][t] == -1000))

    def check_smt(self):
        TC = []
        TR = []
        for r in range(self.R):
            TC+= self.C[r]
            TR+= self.Re[r]
        self.total_cost = Sum(TC)
        self.total_reward = Sum(TR)
        self.constraints.append(self.Obj_func == self.W_Cost*self.total_cost - self.W_Reward*self.total_reward)
        constraints = [simplify(c) for c in self.constraints]
        self.s.add(constraints)
        h = self.s.minimize(self.Obj_func)

        check = str(self.s.check())
        return check
    
    def add_visible_cells(self,loc,d):
        (rx,ry) = loc
        xl = int (max (0, rx-d))
        xh = int (min (self.Dx, rx+d+1))
        yl = int (max (0, ry-d))
        yh = int (min (self.Dy, ry+d+1))
        for x in range (xl, xh):
            for y in range (yl, yh):                          # For another condition to select visible cells
                if (map[y][x] == 0.0): # and d < self.vis_dist):
                    self.new_visible_cells.append((x,y))
                    map[y][x] = 0.5    

    def return_all_vars(self):
        global visible_cells
        model = self.s.model()
        next_start_loc = []
        current_traj = []
        self.new_visible_cells = []
        covered_visible_cells = []
        count = 0
        for r in range(self.R):
            bot_traj = []
            for t in range(self.T):
                rx = int (str (model[self.X[r][t]]))
                ry = int (str (model[self.Y[r][t]]))
                
                if map[ry][rx] == 0.5 :
                    covered_visible_cells.append((rx,ry))
                    #cell_age[ry][rx] = 0
                    count+=1
                if(t>0):
                    bot_traj.append((rx,ry))
                    self.add_visible_cells([rx,ry],self.vis_dist)
                if(t==self.T-1):
                    next_start_loc.append((rx,ry))

                map[ry][rx] = 1.0

            current_traj.append(bot_traj)

        filtered_cells = []
        for cell in visible_cells:
            if cell not in covered_visible_cells:
                filtered_cells.append(cell)

        visible_cells = filtered_cells + self.new_visible_cells
        return next_start_loc,current_traj,count
    def surroundings(self,loc,d=1):
        (vx,vy) = (int(loc[0]),int(loc[1]))
        cov = 0
        vis = 0
        n = self.Dx
        m = self.Dy
        for x in range (vx-d, vx+d+1):
            for y in range (vy-d, vy+d+1): 
                if (x==vx and y==vy):
                    continue                         
                if(x<0 or y<0 or x>=n or y>=m):
                    cov+=1
                else:
                    if map[y][x]==1:
                        cov+=1

        return cov
#------------------------------------------------------------------------------------------------------------------------

def Init_visible_cells(init_loc,Dx,Dy,d):
    global map
    global visible_cells
    visible_cells = []
    for loc in init_loc:
        (rx,ry) = loc
        xl = int (max (0, rx-d))
        xh = int (min (Dx, rx+d+1))
        yl = int (max (0, ry-d))
        yh = int (min (Dy, ry+d+1))
        for x in range (xl, xh):
            for y in range (yl, yh):
                # d = ((rx-x)^2+(ry-y)^2)^(0.5)                           # For another condition to select visible cells
                if (map[y][x] == 0.0): # and d < self.vis_dist):
                    visible_cells.append((x,y))
                    map[y][x] = 0.5 
    

# def update_age():
#     for cell in visible_cells:
#         (x,y) = cell
#         cell_age[y][x]+=1



def make_map(R,Dx,Dy,init_pos,obst_pos):
    global map
    global visited_map
    map  = np.full ((Dy,Dx),0.0)
    visited_map =  np.zeros((Dy,Dx), dtype=bool)
    for pos in init_pos:
        (x,y) = pos
        map[y,x] = 1.0
    for pos in obst_pos:
        (x,y) = pos
        map[y,x] = -1.0


def main(R,T,Dx,Dy,plots_dir,wp_dir,init_pos,obst_pos,vis = False):

    # num_obst = int (Dx*Dy/10)
    # obst_pos = [] # random_obst_positions (Dx, Dy, num_obst)
    # init_pos,map = random_init_positions (Dx, Dy, R, obst_pos)
    make_map(R,Dx,Dy,init_pos,obst_pos)
    vis_dist = 1
    cells_need_to_be_covered = Dx*Dy-len(obst_pos)
    cells_covered = R
    n_neighbors = 5
    Init_visible_cells(init_pos,Dx,Dy,vis_dist)
    #global cell_age
    # cell_age =  np.full ((Dy,Dx), 0)
    # update_age()
    
    files = []
    for r in range (R):
        filename = 'robot_' + str (r)
        filepath = os.path.join(wp_dir, filename)
        f = open (filepath, 'w+')
        files.append (f)
        x,y = init_pos[r]
        s = str (y) + " " + str (x) + "\n"
        files[r].write (s)
        files[r].flush ()
        
    
    k=0
    total_time = 0
    while True:
        
        if cells_covered>=cells_need_to_be_covered and len(visible_cells)==0:
            break
        k+=1
        
        tic = time.time()

        smt = SMT(R,T,Dx,Dy,vis_dist,n_neighbors)
        smt.make_and_assign_clusters(init_pos)

        toc1 = time.time()

        smt.init_bot_loc(init_pos)
        #smt.motion_primitive()
        smt.new_collision_avoidance()
        #smt.obstacle_avoidance(obst_pos)

        toc2 = time.time()

        smt.visit_reachable_cells()

        toc3 = time.time()
        
        if(smt.check_smt()=='unsat'):
            break
        tocF = time.time()
        dt1 = round(toc1 - tic,3)
        dt2 = round(toc2 - toc1,3)
        dt3 = round(toc3 - toc2,3)
        dt4 = round(tocF - toc3,3)
        dt = round(tocF - tic,3)
        total_time+= dt

        init_pos,current_traj,count = smt.return_all_vars()
        cells_covered+=count
        no_visible = len(visible_cells)
        no_uncovered_cells = Dx*Dy -cells_covered-no_visible
        print("For horizon {} : Total time taken : {} sec , Total cells covered : {} , Visible cells : {} , Uncovered cells : {}\n".format(k,dt,cells_covered,no_visible,no_uncovered_cells))
        if dt<60:
            print("                 Total time taken : {} sec, cluster  : {} sec, collision : {} sec, visit_reach : {} sec, SMT_check : {} sec\n".format(dt,dt1,dt2,dt3,dt4))
        else:
            print("                 Total time taken : {} min, cluster  : {} min, collision : {} min, visit_reach : {} sec, SMT_check : {} min\n".format(dt/60,dt1/60,dt2/60,dt3/60,dt4/60))
        # #update_age()
        
        for r in range(R):
            for loc in current_traj[r]:
                x,y = loc
                s = str (y) + " " + str (x) + "\n"
                files[r].write (s)
                files[r].flush ()
        

    if (cells_covered<cells_need_to_be_covered):
        print("SMT not Satisfied")
    print("Total no of horizons needed : {} \n".format(k))
    if total_time<60:
        print("Total Time taken   : {} sec\n".format(total_time))
    else:
        print("Total Time taken   : {} min\n".format(total_time/60))
    
    return k,round(total_time,3)




import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-r', dest='num_robots', type=int, help='Number of robots')
parser.add_argument('-d', dest='dimension', type=int, help='Size of workspace')
parser.add_argument('-t', dest='tests', type=int, help='No of tests')
parser.add_argument('-it', default=1, dest='init_test', type=int, help='Initial test location')
parser.add_argument('-v', dest='vis', type=int, help='should visualize or not')
parser.add_argument('-f', dest='filename', type=str, help='Name of the file to save')   
args = parser.parse_args()
D = int(args.dimension)
R = int(args.num_robots)
it = int(args.init_test)
filename = str(D)+'x'+str(D)+'_'+str(R)+'bots'

init_loc_file = "INITIAL_LOCATIONS-"+str(3)

if(args.filename==""):
   filename = args.filename

if not os.path.isdir (filename):
    os.mkdir (filename)

Dx = D
Dy = D
T = 2 

do_test = 0
vis = False
if args.vis == 1:
    vis = True

if(args.tests):
    do_test = args.tests

if (do_test==0):
    test_dir = os.path.join(filename,'TEST'+str(it))
    if not os.path.isdir (test_dir):
        os.mkdir (test_dir)
    plots_dir = os.path.join(test_dir, 'plots')
    if not os.path.isdir (plots_dir):
        os.mkdir (plots_dir)

    wp_dir = os.path.join(test_dir, 'WPts')
    if not os.path.isdir (wp_dir):
        os.mkdir (wp_dir)
    path = os.path.join(init_loc_file,str(D)+'x'+str(D)+'_'+str(R),'TEST-'+str(it),'robot_init_locs')
    file = open(path,'r')
    init_pos = []
    obst_pos = []
    for r in range(R):
        NewLine = file.readline()
        y,x = int (NewLine.split(' ')[0]), int (NewLine.split(' ')[1])
        init_pos.append((x,y))
    k,total_time =  main(R,T,Dx,Dy,plots_dir,wp_dir,init_pos,obst_pos,vis)
#######################################
else:
    tests = do_test
    Avg_k = 0
    Avg_time = 0
    K = []
    Time = []
    for i in range(tests):
        print("TEST : ",i+1)
        test_dir = os.path.join(filename,'TEST'+str(i+1))
        if not os.path.isdir (test_dir):
            os.mkdir (test_dir)
        plots_dir = os.path.join(test_dir, 'plots')
        if not os.path.isdir (plots_dir):
            os.mkdir (plots_dir)

        wp_dir = os.path.join(test_dir, 'WPts')
        if not os.path.isdir (wp_dir):
            os.mkdir (wp_dir)
        path = os.path.join(init_loc_file,str(D)+'x'+str(D)+'_'+str(R),'TEST-'+str(i+1),'robot_init_locs')
        file = open(path,'r')
        init_pos = []
        for r in range(R):
            NewLine = file.readline()
            y,x = int (NewLine.split(' ')[0]), int (NewLine.split(' ')[1])
            init_pos.append((x,y))
        k,total_time =  main(R,T,Dx,Dy,plots_dir,wp_dir,init_pos,obst_pos=[])
        K.append(k)
        Time.append(total_time)

    for i in range(tests):
        if (Time[i]< 60):
            print("TEST {} -------------->   No of horizons : {}  ,Computation time : {} sec\n".format(i+1,K[i],Time[i]))
        else:
            print("TEST {} -------------->   No of horizons : {}  ,Computation time : {} min\n".format(i+1,K[i],Time[i]/60))
            
    Avg_k = mean(K)
    Avg_time = mean(Time)
    sd_k = stdev(K)
    sd_time = stdev(Time)
    print("For {}x{} grid & {} robots in {} tests ------>>>> Average no of horizons needed  : {} , Standard Deviation : {}\n".format(D,D,R,tests,Avg_k,sd_k))

    if (Avg_time< 60):
        print("For {}x{} grid & {} robots in {} tests ------>>>> Average no of horizons needed  : {} sec, Standard Deviation : {} sec\n".format(D,D,R,tests,Avg_time,sd_time))
    else:
        print("For {}x{} grid & {} robots in {} tests ------>>>> Average Computation time needed  : {} min, Standard Deviation : {} sec\n".format(D,D,R,tests,Avg_time/60,sd_time))