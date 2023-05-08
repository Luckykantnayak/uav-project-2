import pygame
from math import *
from queue import PriorityQueue
import time
from random import randrange as rand

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0,0,139)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)
gray90 = (229,229,229)
sepia = (94,38,18)
maroon3 = (205,41,144)
class Spot:
	def __init__(self, row, col, width, total_rows):
		self.row = row
		self.col = col
		self.x = row * width
		self.y = col * width
		self.state = gray90
		self.visible_cells = []
		self.width = width
		self.total_rows = total_rows

	def get_pos(self):
		return self.row, self.col

	def covered(self):
		self.state = GREEN

	def robot(self,colour = PURPLE):
		self.state = colour


	def is_barrier(self):
		return self.state == BLACK
    
	
	def draw(self, win):
		pygame.draw.rect(win, self.state, (self.x, self.y, self.width, self.width))

	def update_visible(self,grid,l=1):
		x_ = self.row
		y_ = self.col
		dimension_x = self.total_rows
		dimension_y = self.total_rows
		x_l = int (max (0, x_-l))
		x_h = int (min (dimension_x, x_+l+1))
		y_l = int (max (0, y_-l))
		y_h = int (min (dimension_y, y_+l+1))
		for x in range (x_l, x_h):
			for y in range (y_l, y_h):
				if grid[x][y].state == gray90:
					grid[x][y].state = YELLOW
		return grid
	# def update_neighbors(self, grid):
	# 	self.neighbors = []
	# 	if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): # DOWN
	# 		self.neighbors.append(grid[self.row + 1][self.col])

	# 	if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): # UP
	# 		self.neighbors.append(grid[self.row - 1][self.col])

	# 	if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # RIGHT
	# 		self.neighbors.append(grid[self.row][self.col + 1])

	# 	if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # LEFT
	# 		self.neighbors.append(grid[self.row][self.col - 1])

	def __lt__(self, other):
		return False




def make_grid(rows, width):
	grid = []
	gap = width / rows
	for i in range(rows):
		grid.append([])
		for j in range(rows):
			spot = Spot(i, j, gap, rows)
			grid[i].append(spot)

	return grid


def draw_grid(win, rows, width):
	gap = width / rows
	for i in range(rows):
		pygame.draw.line(win, BLACK, (0, i * gap), (width, i * gap))
		for j in range(rows):
			pygame.draw.line(win, BLACK, (j * gap, 0), (j * gap, width))


def draw(win, grid, rows, width):
	win.fill(gray90)

	for row in grid:
		for spot in row:
			spot.draw(win)

	draw_grid(win, rows, width)
	pygame.display.update()


import os
def main(win, width,R,D,test):
	ROWS = D
	
	grid = make_grid(ROWS, width)
	clr = [PURPLE,RED,BLUE,sepia]#,ORANGE,TURQUOISE]
	c_len = len(clr)
	Rclr = []
	for i in range(R):
		Rclr.append(clr[rand(c_len)])
	files = []
	Prev_row = []
	Prev_col = []
	for r in range(R):
		#path = os.path.join(str(D)+'x'+str(D)+'_'+str(R)+'bots','TEST'+str(test),'WPts','robot_'+str(r))
		path = os.path.join(str(D)+'x'+str(D)+'_'+str(R)+"-1",'TEST-'+str(test),'robot_'+str(r))
		files.append(open(path,'r'))
		NewLine = files[r].readline()
		row,col = D-1-int (NewLine.split(' ')[0]), D-1-int (NewLine.split(' ')[1])
		Prev_row.append(row)
		Prev_col.append(col)
		spot = grid[row][col]
		spot.robot(Rclr[r])
		grid = spot.update_visible(grid)
	
	while True:
		draw(win, grid, ROWS, width)
		
		line_check = False

		for r in range(R):
			spot = grid[Prev_row[r]][Prev_col[r]]
			spot.covered()
		for r in range(R):
			NewLine = files[r].readline()
			if len(NewLine)>0:
				line_check = True
				row,col = D-1-int (NewLine.split(' ')[0]), D-1-int (NewLine.split(' ')[1])
				#row,col = int (NewLine.split(' ')[0]), int (NewLine.split(' ')[1])
				spot = grid[row][col]
				spot.robot(Rclr[r])
				grid = spot.update_visible(grid)
				Prev_row[r] = row
				Prev_col[r] = col
			else:
				line_check = False
				break
		if(line_check==False):
			break

		time.sleep(0.01)

			

	pygame.quit()

import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-r', dest='num_robots', type=int, help='Number of robots')
parser.add_argument('-d', dest='dimension', type=int, help='Size of workspace')
parser.add_argument('-t', default=1, dest='test', type=int, help='test no')
args = parser.parse_args()
R = int(args.num_robots)
D = int(args.dimension)
test = int(args.test)

WIDTH = 2000 # min(2000,max(500,sqrt(D)*300))
HEIGHT = 2000 #min(2000,max(500,sqrt(D)*300))

WIN = pygame.display.set_mode((WIDTH,HEIGHT))
pygame.display.set_caption("UAV Coverage")

main(WIN, WIDTH ,R,D,test)