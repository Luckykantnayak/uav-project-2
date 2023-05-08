import numpy as np
import faulthandler
import time

faulthandler.enable()
# // bad code goes here
import sys
sys.setrecursionlimit(10**6)

def union_find_components(grid):
    def find(x):
        if parent[x] != x:
            parent[x] = find(parent[x])
        return parent[x]

    def union(x, y):
        px, py = find(x), find(y)
        parent[px] = py

    # Initialize the parent array and set each cell to be its own parent
    rows, cols = len(grid), len(grid[0])
    parent = [i for i in range(rows*cols)]

    # Iterate through the grid and join adjacent cells that contain 1s
    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 1:
                for x, y in [(i-1, j), (i+1, j), (i, j-1), (i, j+1)]:
                    if 0 <= x < rows and 0 <= y < cols and grid[x][y] == 1:
                        union((i*cols) + j, (x*cols) + y)

    # Count the number of unique parents
    component = {}
    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 1:
                p = find((i*cols) + j)
                if p in component:
                    component[p].append((i, j))
                else:
                    component[p] = [(i, j)]
    return list(component.values())

def bfs_find_components(grid):
    # Initialize a visited grid to keep track of which cells have been visited
    visited = [[False for _ in range(len(grid[0]))] for _ in range(len(grid))]

    def bfs(i, j):
        component = []
        # Create a queue to store the cells to visit
        queue = [(i, j)]
        while queue:
            x, y = queue.pop(0)
            if not visited[x][y]:
                component.append((x, y))
                visited[x][y] = True
                # Check the four neighboring cells
                for x, y in [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]:
                    if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 1:
                        queue.append((x, y))
        return component
    components = []
    for i in range(len(grid)):
        for j in range(len(grid[0])):
            if grid[i][j] == 1 and not visited[i][j]:
                component = bfs(i, j)
                if component:
                    components.append(component)
    return components

grid = np.ones((200,200))

tic = time.time()
components = bfs_find_components(grid)
toc = time.time()
print(toc-tic)