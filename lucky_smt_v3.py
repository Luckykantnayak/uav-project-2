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
import copy


class SMT:
    def __init__(self,R,T,Dx,Dy,vis_dist,n_neighbors):
        self.s = Optimize()
        self.R = R
        self.T = T
        self.Dx = Dx
        self.Dy = Dy
        self.vis_dist = vis_dist
        self.n_neighbors = n_neighbors
        self.W_Cost  = 1000 # 100
        self.W_Reward = 1000 # 10
        self.WR_Cov = max(Dx,Dy)*(10+0.1) # 300
        self.WR_Vis = max(Dx,Dy)*(10+0.1) # 300
        self.reachable_cells = [[] for i in range(R)]
        self.X = [[Int("x%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
        self.Y = [[Int("y%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
        self.P = [[Int("p_%s_%s" % (i, j)) for j in range(T-1)] for i in range(R)]
        self.C = [[Real("c_%s_%s" % (i, j)) for j in range(T-1)] for i in range(R)]
        self.Re = [[Real("re_%s_%s" % (i, j)) for j in range(T-1)] for i in range(R)]
        self.total_cost = Real('total_cost')
        self.total_reward = Real('total_reward')
        self.Obj_func = Real('Obj_func')

    def init_bot_loc(self,bot_loc):
        self.bot_loc = bot_loc
        
        for r in range (self.R):
            (x,y) = bot_loc[r]
            self.s.add (And (self.X[r][0] == int (x),   self.Y[r][0] == int (y)))              # Assign the initial x and y coordinates 
            self.collect_reachables(r,self.vis_dist)                                 # collect other reachable locations available  

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
                self.s.add (And (self.P[r][t] <= 4, self.P[r][t] >= 0))  # Only 5 motion primitives are allowed
                self.s.add (Or (self.C[r][t] == 1 , self.C[r][t] == 3 )) # Only 2 cost values are  allowed 

                # For robot r at time t  , If we choose an allowed value of P then the corresponding cost and next state allowed is defined
                self.s.add(Implies(self.P[r][t] == 0, And(self.X[r][t+1] == self.X[r][t],  self.Y[r][t+1] == self.Y[r][t],  self.C[r][t] == 3))) # same
                self.s.add(Implies(self.P[r][t] == 1, And(self.X[r][t+1] == self.X[r][t]+1,self.Y[r][t+1] == self.Y[r][t],  self.C[r][t] == 1))) # right
                self.s.add(Implies(self.P[r][t] == 2, And(self.X[r][t+1] == self.X[r][t]-1,self.Y[r][t+1] == self.Y[r][t],  self.C[r][t] == 1))) # left
                self.s.add(Implies(self.P[r][t] == 3, And(self.X[r][t+1] == self.X[r][t],  self.Y[r][t+1] == self.Y[r][t]+1,self.C[r][t] == 1))) # up
                self.s.add(Implies(self.P[r][t] == 4, And(self.X[r][t+1] == self.X[r][t],  self.Y[r][t+1] == self.Y[r][t]-1,self.C[r][t] == 1))) # down             
    
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
            cov = self.surroundings(next_loc,2)
            return (cell_age[ny][nx]+cov)*self.WR_Vis  
        elif(map[ny][nx] == 1.0):
            return self.near(r,current_loc, next_loc)*self.WR_Cov
        else:
            return -1000
    
    
    ##########
    def near(self,r,current_loc,next_loc):
        (nx,ny) = (next_loc[0],next_loc[1])
        (rx,ry) = (current_loc[0],current_loc[1]) 
        if(len(visible_cells)==0):
            return 0
        
        np_visible_cells = copy.copy(visible_cells)
        np_visible_cells = np.array (np_visible_cells)
        
        dist = abs (np_visible_cells[:,0] - rx) + abs (np_visible_cells[:,1] - ry)
        idxs = dist.argsort ()[:self.n_neighbors]
        safe_visible_cells = np_visible_cells[idxs]
        k = len(safe_visible_cells)
        total_d = 0
        for loc in safe_visible_cells:
           d = abs (loc[0] - nx) + abs (loc[1] - ny)
           total_d += d 
        return k/total_d 
    # def near(self,r,current_loc,next_loc):
    #     (nx,ny) = (next_loc[0],next_loc[1])
    #     (rx,ry) = (current_loc[0],current_loc[1]) 

    #     total_w_d = 0
        
    #     k = len(self.nearest_vis_cells[r])
    #     Total_W = 0
    #     for loc in self.nearest_vis_cells[r]: 
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
        self.nearest_vis_cells = []
        np_visible_cells = copy.copy(visible_cells)
        np_visible_cells = np.array (np_visible_cells)
        for r in range(self.R):
            (rx,ry) = self.bot_loc[r]
            self.nearest_vis_cells.append([])
           
            if(len(np_visible_cells)==0):
                
                continue

            dist = abs (np_visible_cells[:,0] - rx) + abs (np_visible_cells[:,1] - ry)
            idxs = dist.argsort ()[:self.n_neighbors]
            self.nearest_vis_cells[r].append(np_visible_cells[idxs])
            for cell in self.nearest_vis_cells[r]:
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
                    self.s.add (Not( And(self.X[r1][t+1] == self.X[r2][t+1], self.Y[r1][t+1] == self.Y[r2][t+1])))
                    # Head on collision or Swaping position collision
                    self.s.add(Not (And(And(self.X[r1][t+1] == self.X[r2][t],self.Y[r1][t+1] == self.Y[r2][t]),And(self.X[r2][t+1] == self.X[r1][t],self.Y[r2][t+1] == self.Y[r1][t]))))
    
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
                        
                        self.s.add (Not( And(self.X[r1][t+1] == self.X[r2][t+1], self.Y[r1][t+1] == self.Y[r2][t+1])))
                            
                        # Head on collision or Swaping position collision
                        if(self.d_bots(r1,r2)==1):
                            self.s.add(Not (And(And(self.X[r1][t+1] == self.X[r2][t],self.Y[r1][t+1] == self.Y[r2][t]),And(self.X[r2][t+1] == self.X[r1][t],self.Y[r2][t+1] == self.Y[r1][t]))))    
    def obstacle_avoidance(self,obst_loc):
        self.obst_loc = obst_loc
        for r in range (self.R):
            for t in range (1,self.T):
                for obst in obst_loc:
                    # Both the x & y coordinates of r at time t cannot be equal to that of obstacle coordinates
                    self.s.add (Not( And (self.X[r][t] == obst[0], self.Y[r][t] == obst[1])))
                # stay within  the grid bounds
                self.s.add (And (self.X[r][t] < self.Dx, self.X[r][t] >= 0))
                self.s.add (And (self.Y[r][t] < self.Dy, self.Y[r][t] >= 0))

    def visit_reachable_cells(self):
        #self.Visible_cells_common_count()
        for r in range (self.R):
            for t in range (self.T-1):  
                # A robot r at time t must choose  a cell from all the reachable cells
                self.s.add (Or ([And (self.X[r][t+1] == x, self.Y[r][t+1] == y) for (x,y) in self.reachable_cells[r]])) 
                curr = self.bot_loc[r]
                for next in self.reachable_cells[r]:
                    cx,cy = curr
                    nx,ny = next
                    self.s.add(Implies(And(And (self.X[r][t] == int(cx), self.Y[r][t] == int(cy)),And (self.X[r][t+1] == int(nx), self.Y[r][t+1] == int(ny))),And(self.Re[r][t] == self.reward(r,[cx,cy],[nx,ny]),self.C[r][t] == self.action_cost([cx,cy],[nx,ny]))))
                    #self.s.add(Implies(Or(Not(And (self.X[r][t] == int(cx), self.Y[r][t] == int(cy))),Not(And (self.X[r][t+1] == int(nx), self.Y[r][t+1] == int(ny)))),self.Re[r][t] == -1000))

    def check_smt(self):
        TC = []
        TR = []
        for r in range(self.R):
            TC+= self.C[r]
            TR+= self.Re[r]
        self.total_cost = Sum(TC)
        self.total_reward = Sum(TR)
        self.s.add(self.Obj_func == self.W_Cost*self.total_cost - self.W_Reward*self.total_reward)
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
                    cell_age[ry][rx] = 0
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
    

def update_age():
    for cell in visible_cells:
        (x,y) = cell
        cell_age[y][x]+=1
   

def get_rec (x, y, t, alpha):
    color = ''
    c = ['r','m','b','k']
    n = len(c)
    if t == 'covered':
        color = 'lawngreen'
    elif t == 'visible':
        color = 'yellow'
    elif t == 'obstacle':
        color = 'black'
    elif t == 'positions':
        color = c[rand(n)]
    else:
        color = 'gray'

    rec = Rectangle ((x,y), 1, 1, linewidth=1, edgecolor='black', facecolor=color, alpha=alpha)
    return rec

def draw_rec (ax, loc, t, alpha):
    x,y = (loc[0],loc[1])
    rect = get_rec (x, y, t, alpha)
    ax.add_patch (rect)

def visualize (ax, init_pos, alpha):
    global map
    dimension_x = map.shape[1]
    dimension_y = map.shape[0]
    for r in range(dimension_y):
        for c in range(dimension_x):
            if   map[r][c] == 0.5:
                draw_rec (ax, [c,r], 'visible', alpha)
            elif map[r][c] == 1.0:
                if (c,r) in init_pos:
                    draw_rec (ax, [c,r], 'positions', alpha)
                else:
                    draw_rec (ax, [c,r], 'covered', alpha)
            elif map[r][c] == -1.0:
                draw_rec (ax, [c,r], 'obstacle', alpha)
            else:
                draw_rec (ax, [c,r], 'uncovered', alpha)

def make_map(R,Dx,Dy,init_pos,obst_pos):
    global map
    map = np.full ((Dy,Dx),0.0)
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
    
    vis_dis = 1
    cells_need_to_be_covered = Dx*Dy-len(obst_pos)
    cells_covered = R
    n_neighbors = 5
    Init_visible_cells(init_pos,Dx,Dy,vis_dis)
    global cell_age
    cell_age =  np.full ((Dy,Dx), 0)
    update_age()
    
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
        
    
    dimension_x = map.shape[1]
    dimension_y = map.shape[0]
    fig, ax = plt.subplots (figsize=(min(25,dimension_x),min(25,dimension_y)))
    ax.set (xlim=(0,dimension_x), ylim=(0,dimension_y))
    ax.axis ('off')
    alpha = 1
    k=0
    total_time = 0
    while True:
        if vis:
            visualize(ax,init_pos,alpha)
            plt.pause (0.001)
            plt.savefig (plots_dir + '/plot' + str (k) + '.png')
        
        if cells_covered>=cells_need_to_be_covered and len(visible_cells)==0:
            break
        k+=1
        
        tic = time.time()

        smt = SMT(R,T,Dx,Dy,vis_dis,n_neighbors)
        smt.init_bot_loc(init_pos)
        #smt.motion_primitive()
        smt.new_collision_avoidance()
        #smt.obstacle_avoidance(obst_pos)
        smt.visit_reachable_cells()

        if(smt.check_smt()=='unsat'):
            break

        toc = time.time()
        dt = toc-tic
        total_time+= dt

        init_pos,current_traj,count = smt.return_all_vars()
        cells_covered+=count
        no_visible = len(visible_cells)
        no_uncovered_cells = Dx*Dy -cells_covered-no_visible
        if dt<60:
            print("Time taken for horizon {} : {} sec, Total cells covered : {} , Visible cells : {} , Uncovered cells : {}\n".format(k,dt,cells_covered,no_visible,no_uncovered_cells))
        else:
            print("Time taken for horizon {} : {} min, Total cells covered : {} , Visible cells : {} , Uncovered cells : {}\n".format(k,dt/60,cells_covered,no_visible,no_uncovered_cells))
        update_age()
        
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
    
    return k,total_time




# import argparse

# parser = argparse.ArgumentParser()
# parser.add_argument('-r', dest='num_robots', type=int, help='Number of robots')
# parser.add_argument('-d', dest='dimension', type=int, help='Size of workspace')
# parser.add_argument('-t', dest='tests', type=int, help='No of tests')
# parser.add_argument('-it',default=1, dest='init_test', type=int, help='Initial test location')
# parser.add_argument('-v', dest='vis', type=int, help='should visualize or not')
# parser.add_argument('-f', dest='filename', type=str, help='Name of the file to save')   
# args = parser.parse_args()
# D = int(args.dimension)
# R = int(args.num_robots)
# it = int(args.init_test)
# filename = str(D)+'x'+str(D)+'_'+str(R)+'bots'

# init_loc_file = "INITIAL_LOCATIONS-"+str(1)

# if(args.filename==""):
#    filename = args.filename

# if not os.path.isdir (filename):
#     os.mkdir (filename)

# Dx = D
# Dy = D
# T = 2 

# do_test = 0
# vis = False
# if args.vis == 1:
#     vis = True

# if(args.tests):
#     do_test = args.tests

# if (do_test==0):
#     test_dir = os.path.join(filename,'TEST'+str(it))
#     if not os.path.isdir (test_dir):
#         os.mkdir (test_dir)
#     plots_dir = os.path.join(test_dir, 'plots')
#     if not os.path.isdir (plots_dir):
#         os.mkdir (plots_dir)

#     wp_dir = os.path.join(test_dir, 'WPts')
#     if not os.path.isdir (wp_dir):
#         os.mkdir (wp_dir)

#     path = os.path.join(init_loc_file,str(D)+'x'+str(D)+'_'+str(R),'TEST-'+str(it),'robot_init_locs')
#     file = open(path,'r')
#     init_pos = []
#     obst_pos = []
#     for r in range(R):
#         NewLine = file.readline()
#         y,x = int (NewLine.split(' ')[0]), int (NewLine.split(' ')[1])
#         init_pos.append((x,y))
#     k,total_time =  main(R,T,Dx,Dy,plots_dir,wp_dir,init_pos,obst_pos,vis)
# #######################################
# else:
#     tests = do_test
#     Avg_k = 0
#     Avg_time = 0
#     K = []
#     Time = []
#     for i in range(tests):
#         print("TEST : ",i+1)
#         test_dir = os.path.join(filename,'TEST'+str(i+1))
#         if not os.path.isdir (test_dir):
#             os.mkdir (test_dir)
#         plots_dir = os.path.join(test_dir, 'plots')
#         if not os.path.isdir (plots_dir):
#             os.mkdir (plots_dir)

#         wp_dir = os.path.join(test_dir, 'WPts')
#         if not os.path.isdir (wp_dir):
#             os.mkdir (wp_dir)
#         path = os.path.join(init_loc_file,str(D)+'x'+str(D)+'_'+str(R),'TEST-'+str(i+1),'robot_init_locs')
#         file = open(path,'r')
#         init_pos = []
#         obst_pos = []
#         for r in range(R):
#             NewLine = file.readline()
#             y,x = int (NewLine.split(' ')[0]), int (NewLine.split(' ')[1])
#             init_pos.append((x,y))
#         k,total_time =  main(R,T,Dx,Dy,plots_dir,wp_dir,init_pos,obst_pos)
#         K.append(k)
#         Time.append(total_time)

#     for i in range(tests):
#         if (Time[i]< 60):
#             print("TEST {} -------------->   No of horizons : {}  ,Computation time : {} sec\n".format(i+1,K[i],Time[i]))
#         else:
#             print("TEST {} -------------->   No of horizons : {}  ,Computation time : {} min\n".format(i+1,K[i],Time[i]/60))
            
#     Avg_k = mean(K)
#     Avg_time = mean(Time)
#     sd_k = stdev(K)
#     sd_time = stdev(Time)
#     print("For {}x{} grid & {} robots in {} tests ------>>>> Average no of horizons needed  : {} , Standard Deviation : {}\n".format(D,D,R,tests,Avg_k,sd_k))

#     if (Avg_time< 60):
#         print("For {}x{} grid & {} robots in {} tests ------>>>> Average no of horizons needed  : {} sec, Standard Deviation : {} sec\n".format(D,D,R,tests,Avg_time,sd_time))
#     else:
#         print("For {}x{} grid & {} robots in {} tests ------>>>> Average Computation time needed  : {} min, Standard Deviation : {} sec\n".format(D,D,R,tests,Avg_time/60,sd_time))
