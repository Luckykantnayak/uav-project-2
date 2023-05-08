from random import random
import numpy as np
import time
from math import *
import os
import sys
sys.setrecursionlimit(10**6)

clusters = []
visible_cells = []
class Cluster:
    def __init__(self,m,n):
        # Get the dimensions of the grid
        self.rows = m
        self.cols = n
        self.visited_map =  np.zeros((m,n), dtype=bool)
        global clusters
        clusters = []
        
    

    def traverse(self,r, c ):
        # Check if the current cell is out of bounds or has already been visited
        if r < 0 or r >= self.rows or c < 0 or c >= self.cols or self.visited_map[r][c]:
            return
        
        # Check if the current cell is a 0
        if map[r][c] != 0.5:
            return
        
        # Mark the current cell as visited
        self.visited_map[r][c] = True
        self.component.append((c,r))
        
        
        # Recursively traverse the neighbors of the current cell
        self.traverse(r + 1, c)  # right
        self.traverse(r - 1, c)  # left
        self.traverse(r, c + 1)  # down
        self.traverse(r, c - 1)  # up
    
    def make_clusters(self):
        for  (x,y) in visible_cells:
            (r,c) = (y,x)
            # Skip cells that have already been visited
            if self.visited_map[r][c]:
                continue
            # Initialize a new connected component as a list of coordinates
            self.component = []
            # Traverse the connected component and add the coordinates of each cell to the list
            self.traverse(r, c )
            # Add the connected component to the list of components
            if self.is_Hole(self.component):
               clusters.append(np.array(self.component))
            
        
    def is_Hole(self, component):
        # Get the dimensions of the map
        rows = len(map)
        cols = len(map[0])
        
        visited_map = np.zeros((rows,cols), dtype=bool)
        # Initialize a list to store the neighboring 0s of the component
        covered = []
        unexp = []
        for cell in component:
            (r, c) = cell
            # Check the neighbors of the current cell   
            if r > 0 and r < rows - 1 and c > 0 and c < cols - 1 :
                if map[r - 1][c] == 1.0 and (not visited_map [r-1][c]):              # if the neighbouring cell is covered then append
                    visited_map [r-1][c] = True
                    covered.append((r - 1, c))
                elif map[r - 1][c] == 0.0 and (not visited_map [r-1][c]):              # if the neighbouring cell is covered then append
                    visited_map [r-1][c] = True
                    unexp.append((r - 1, c))

                if  map[r + 1][c] == 1.0 and (not visited_map [r+1][c]):              # if the neighbouring cell is covered then append
                    visited_map [r+1][c] = True
                    covered.append((r + 1, c))
                elif map[r + 1][c] == 0.0 and (not visited_map [r+1][c]):              # if the neighbouring cell is covered then append
                    visited_map [r+1][c] = True
                    unexp.append((r+1, c))    

                if map[r][c - 1] == 1.0 and (not visited_map [r][c-1]):              # if the neighbouring cell is covered then append
                    visited_map [r][c-1] = True
                    covered.append((r, c - 1))
                elif map[r][c-1] == 0.0 and (not visited_map [r][c-1]):              # if the neighbouring cell is covered then append
                    visited_map [r][c-1] = True
                    unexp.append((r, c-1))

                if  map[r][c + 1] == 1.0 and (not visited_map [r][c+1]):              # if the neighbouring cell is covered then append
                    visited_map [r][c+1] = True
                    covered.append((r, c + 1))
                elif map[r][c+1] == 0.0 and (not visited_map [r][c+1]):              # if the neighbouring cell is covered then append
                    visited_map [r][c+1] = True
                    unexp.append((r, c+1))
                


                if map[r - 1][c-1] == 1.0 and (not visited_map [r-1][c-1]):              # if the neighbouring cell is covered then append
                    visited_map [r-1][c-1] = True
                    covered.append((r - 1, c-1))
                elif map[r - 1][c-1] == 0.0 and (not visited_map [r-1][c-1]):              # if the neighbouring cell is covered then append
                    visited_map [r-1][c-1] = True
                    unexp.append((r - 1, c-1))

                if  map[r + 1][c+ 1] == 1.0 and (not visited_map [r+1][c+ 1]):              # if the neighbouring cell is covered then append
                    visited_map [r+1][c+ 1] = True
                    covered.append((r + 1, c+ 1))
                elif map[r + 1][c+ 1] == 0.0 and (not visited_map [r+1][c+ 1]):              # if the neighbouring cell is covered then append
                    visited_map [r+1][c+ 1] = True
                    unexp.append((r+1, c+ 1))    

                if map[r+ 1][c - 1] == 1.0 and (not visited_map [r+ 1][c-1]):              # if the neighbouring cell is covered then append
                    visited_map [r+ 1][c-1] = True
                    covered.append((r+ 1, c - 1))
                elif map[r+ 1][c-1] == 0.0 and (not visited_map [r+ 1][c-1]):              # if the neighbouring cell is covered then append
                    visited_map [r+ 1][c-1] = True
                    unexp.append((r+ 1, c-1))

                if  map[r- 1][c + 1] == 1.0 and (not visited_map [r- 1][c+1]):              # if the neighbouring cell is covered then append
                    visited_map [r- 1][c+1] = True
                    covered.append((r- 1, c + 1))
                elif map[r- 1][c+1] == 0.0 and (not visited_map [r- 1][c+1]):              # if the neighbouring cell is covered then append
                    visited_map [r- 1][c+1] = True
                    unexp.append((r- 1, c+1))
            else:                                       # if it is a boundary cell return false
                return False
        
        # Check if there are any covered in the list
        
        return len(unexp)<len(covered)
        
        

def update_visible(row,col,D,l=1):
    r_ = row
    c_ = col
    dimension_r = D
    dimension_c = D
    
    r_l = int (max (0, r_-l))
    r_h = int (min (dimension_r, r_+l+1))
    c_l = int (max (0, c_-l))
    c_h = int (min (dimension_c, c_+l+1))
    for r in range (r_l, r_h):
        for c in range (c_l, c_h):
            if map[r][c] == 0.0:
                map[r][c] = 0.5
                visible_cells.append((r,c))
                    
		
def main(D,R,test):
    global map
    map  = np.full ((D,D),0.0)
    files = []
    Prev_row = []
    Prev_col = []
    for r in range(R):
        path = os.path.join(str(D)+'x'+str(D)+'_'+str(R)+'bots','TEST'+str(test),'WPts','robot_'+str(r))
        files.append(open(path,'r'))
        NewLine = files[r].readline()
        row,col = int (NewLine.split(' ')[0]), int (NewLine.split(' ')[1])
        Prev_row.append(row)
        Prev_col.append(col)
        update_visible(row,col,D)
    

    while True:
        line_check = False

        for r in range(R):
            (row,col) = Prev_row[r],Prev_col[r]
            map[row][col] = 1.0

        for r in range(R):
            NewLine = files[r].readline()
            if len(NewLine)>0:
                line_check = True
                row,col = int (NewLine.split(' ')[0]), int (NewLine.split(' ')[1])
                update_visible(row,col,D)
                Prev_row[r] = row
                Prev_col[r] = col
            else:
                line_check = False
                break
        if(line_check==False):
            break

        

import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-r', dest='num_robots', type=int, help='Number of robots')
parser.add_argument('-d', dest='dimension', type=int, help='Size of workspace')
parser.add_argument('-t', default=1, dest='test', type=int, help='test no')
args = parser.parse_args()
R = int(args.num_robots)
D = int(args.dimension)
test = int(args.test)




