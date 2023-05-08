from random import randrange as rand
import numpy as np
from math import *
def get_random_init_positions (dimension, R):
    init_pos = []
    grids = np.full ((dimension,dimension), 0.5)
    while len(init_pos)!=R:
        pos = [rand(dimension),rand(dimension)]
        if(grids[pos[1],pos[0]]==0.5):
            init_pos.append(pos)
            grids[pos[1],pos[0]] = 1
        else:
            continue
    
    return init_pos,grids


def get_random_obst_positions (grids, obst):
    num_ob = obst
    obst_pos = []
    dimension = grids.shape[0]
    while len(obst_pos)!=num_ob:
        pos = [rand(dimension),rand(dimension)]
        if(grids[pos[1],pos[0]]==0.5):
            obst_pos.append(pos)
            grids[pos[1],pos[0]] = 0
        else:
            continue
    return obst_pos

def get_print_grid_str (grids, dimension):
    grid= ""
    for i in range(dimension):
        for j in range(dimension):
            if grids[i][j]==1:
                grid+= "1.0 "
            elif grids[i][j]==0:
                grid+= "0.0 "
            else:
                grid+= "0.5 "
        grid+="\n"
    return grid

# For lucky smt --------------------------------------------------------------------------------------------------

def random_init_positions (Dx, Dy, R , obst_pos):
    init_pos = []
    grids = np.full ((Dy,Dx),0.0)
    while len(init_pos)!=R:
        x = rand(Dx) 
        y = rand(Dy)
        pos = (x,y)
        if(pos in obst_pos):
            grids[y,x] = -1.0
        else:
            init_pos.append(pos)
            grids[y,x] = 1.0
    return init_pos,grids

def new_random_init_positions (Dx, Dy, R , obst_pos):
    init_pos = []
    
    while len(init_pos)!=R:
        x = rand(Dx) 
        y = rand(Dy)
        pos = (x,y)
        if pos not in obst_pos and pos not in init_pos:
            init_pos.append(pos)
            
    return init_pos

def get_close_init_positions (Dx, Dy, R , obst_pos):

    # R must be <= Dx and Dy
    #h = R//2 + 1
    h = 1
    by = rand(1,h+1)
    l = int(ceil(R/by))
    bx = rand(l,R+1) if l!=R else l
    cx = rand(0,Dx-bx) if Dx != bx else 0
    cy = 0

    init_pos = []
    
    while len(init_pos) != R:
        x = rand(by+cy)
        y = rand(bx+cx)
        pos = (x,y)
        if pos not in obst_pos and pos not in init_pos:
            init_pos.append(pos)
    
    return init_pos
def random_obst_positions (Dx, Dy, n):
    obst_pos = []
    while len(obst_pos)!=n:
        pos = (rand(Dx),rand(Dy))
        if(pos not in obst_pos):
            obst_pos.append(pos) 
        else:
            continue
    return obst_pos
