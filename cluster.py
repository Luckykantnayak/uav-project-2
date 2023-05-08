#!/usr/bin/env python3

import numpy as np
from collections import deque

class Cluster:
    def __init__(self,m,n,visible_cells):
        self.visible_cells = visible_cells
        
        # Get the dimensions of the grid
        self.rows = m
        self.cols = n
        self.grid = self.make_grid()
        # Create a visited array to keep track of which cells have been visited
        self.visited = [[False for _ in range(self.cols)] for _ in range(self.rows)]
        self.clusters = []
    
    def make_grid(self):
        grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]
        for (x,y) in self.visible_cells:
            grid[y][x] = 1
        return grid

    def traverse(self,r, c):
        # Check if the current cell is out of bounds or has already been visited
        if r < 0 or r >= self.rows or c < 0 or c >= self.cols or self.visited[r][c]:
            return
        
        # Check if the current cell is a 0
        if self.grid[r][c] == 0:
            return
        
        # Mark the current cell as visited
        self.visited[r][c] = True
        self.component.append((c,r))
        
        # Recursively traverse the neighbors of the current cell
        self.traverse(r + 1, c)  # right
        self.traverse(r - 1, c)  # left
        self.traverse(r, c + 1)  # down
        self.traverse(r, c - 1)  # up
    
    def make_clusters(self):
        for (x,y) in self.visible_cells:
            r = y
            c = x
            # Skip cells that have already been visited
            if self.visited[r][c]:
                continue
            
            # If the current cell is a 1, start a new connected component
            if self.grid[r][c] == 1:
                # Initialize a new connected component as a list of coordinates
                self.component = []
                # Traverse the connected component and add the coordinates of each cell to the list
                self.traverse(r, c)
                # Add the connected component to the list of components
                self.clusters.append(self.component)
        
        return self.clusters





    # # Perform breadth-first search on the grid
    # def breadth_first_search(self, start_i, start_j):
    #     # Queue to store the cells to be explored
    #     queue = deque([(start_i, start_j)])

    #     # Set to store the visited cells
    #     visited = []

    #     # Mark the starting cell as visited
    #    # self.mark_visited(start_i, start_j)
    #     visited.append((start_i, start_j))

    #     # Explore the grid in breadth-first order
    #     while queue:
    #         i, j = queue.popleft()
    #         # Explore the neighboring cells
    #         for ni, nj in [(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)]:
    #             if self.is_valid_and_unvisited(ni, nj,visited):
                    
    #                 visited.append((ni, nj))
    #                 queue.append((ni, nj))

    #     self.clusters.append(visited)


    # # Mark a cell as visited
    # def mark_visited(self, i, j):
    #     self.grid[j][i] = 0

    # # Check if a cell is valid and has not been visited
    # def is_valid_and_unvisited(self, i, j,visited):
    #     return (i >= 0 and i < self.m and
    #             j >= 0 and j < self.n and
    #             (i,j) not in visited)

    
    


# def get_connected_components(grid):
#     # Initialize a list to store the connected components
#     components = []
    
#     # Get the dimensions of the grid
#     rows = len(grid)
#     cols = len(grid[0])
    
#     # Create a visited array to keep track of which cells have been visited
#     visited = [[False for _ in range(cols)] for _ in range(rows)]
    
#     # Define a recursive function to traverse the grid and find connected components
#     def traverse(r, c):
#         # Check if the current cell is out of bounds or has already been visited
#         if r < 0 or r >= rows or c < 0 or c >= cols or visited[r][c]:
#             return
        
#         # Check if the current cell is a 0
#         if grid[r][c] == 0:
#             return
        
#         # Mark the current cell as visited
#         visited[r][c] = True
        
#         # Recursively traverse the neighbors of the current cell
#         traverse(r + 1, c)  # right
#         traverse(r - 1, c)  # left
#         traverse(r, c + 1)  # down
#         traverse(r, c - 1)  # up
    
#     # Loop through the grid and find connected components
#     for r in range(rows):
#         for c in range(cols):
#             # Skip cells that have already been visited
#             if visited[r][c]:
#                 continue
            
#             # If the current cell is a 1, start a new connected component
#             if grid[r][c] == 1:
#                 # Initialize a new connected component as a list of coordinates
#                 component = []
#                 # Traverse the connected component and add the coordinates of each cell to the list
#                 traverse(r, c)
#                 # Add the connected component to the list of components
#                 components.append(component)
    
#     # Return the list of connected components
#     return components

