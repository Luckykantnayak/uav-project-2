from math import ceil
from z3 import *
import time
from random import randrange as rand
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-r', dest='num_robots', type=int, help='Number of robots')
parser.add_argument('-d', dest='dimension', type=int, help='Size of workspace')
args = parser.parse_args()

R = int(args.num_robots)
D = int(args.dimension)

dimension_x = D
dimension_y = D

T = ceil(dimension_x*dimension_y/R)+1

s = Optimize()


X = [[Int("x_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
Y = [[Int("y_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]
P = [[Int("p_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]

C = [[Int("c_%s_%s" % (i, j)) for j in range(T)] for i in range(R)]

total_cost = Int ('total_cost')
robot_cost = [Int("robot_cost_%s" % (i)) for i in range(R)]

for r in range (R):
    s.add(robot_cost[r] == Sum(C[r]))

s.add(total_cost == Sum(robot_cost))


#  Random Start Positions
for r in range(R):
    s.add(X[r][0] == rand(dimension_x))
    s.add(Y[r][0] == rand(dimension_y))

    

#obst = [(2,0), (3,0), (1,2), (3,2), (1,4), (2,4)]
obst = []
for r in range(R):
    for t in range(0,T):
        for ob in obst:
            s.add(Or(X[r][t] != ob[0], Y[r][t] != ob[1]))

# Obstacle avoidance
for r in range(R):
    for t in range(0,T):

        # stay within bounds
        s.add(And (X[r][t] < dimension_x, X[r][t] >= 0))
        s.add(And (Y[r][t] < dimension_y, Y[r][t] >= 0))
        s.add(And (P[r][t] <= 4, P[r][t] >= 0))

        s.add (Or (C[r][t] == 1, C[r][t] == 2))

# collision avoidance
for t in range(0, T):
    for r1 in range (R):
        for r2 in range (R):
            if (r1 != r2 and r1 < r2):
                #s.add (Or(X[r1][t] != X[r2][t], Y[r1][t] != Y[r2][t]))
                s.add (And(X[r1][t] != X[r2][t], Y[r1][t] != Y[r2][t]))

# full coverage condition
for x in range(0, dimension_x):
    for y in range(0, dimension_y):
        if ((x,y) not in obst):
            s.add(Or([And(X[r][t] == x, Y[r][t] == y) for r in range(0, R) for t in range (0, T)]))

# Motion primitives
for r in range(0, R):
    for t in range(T-1):
        s.add(Implies(P[r][t] == 0, And(X[r][t+1] == X[r][t], Y[r][t+1] == Y[r][t], C[r][t] == 2))) # same
        s.add(Implies(P[r][t] == 1, And(X[r][t+1] == X[r][t]+1, Y[r][t+1] == Y[r][t], C[r][t] == 1))) # right
        s.add(Implies(P[r][t] == 2, And(X[r][t+1] == X[r][t], Y[r][t+1] == Y[r][t]+1, C[r][t] == 1))) # up
        s.add(Implies(P[r][t] == 3, And(X[r][t+1] == X[r][t], Y[r][t+1] == Y[r][t]-1, C[r][t] == 1))) # down
        s.add(Implies(P[r][t] == 4, And(X[r][t+1] == X[r][t]-1, Y[r][t+1] == Y[r][t], C[r][t] == 1))) # left

h = s.minimize(total_cost)

# for r in range (R):
#     s.minimize(robot_cost[r])

tic = time.time()
print ("Whether the model is satisfiable?: ", s.check())
toc = time.time()
dt = toc - tic
if dt<60:
    print("Computation Time Taken : ",toc-tic," sec")
else:
    print("Computation Time Taken : ",(toc-tic)/60," min")

# print ("============ Solution ================")
model = s.model()
print(model)
print ("Total Cost ",model[total_cost])
for r in range (R):
    print ("Cost of Robot "+str(r)," : ",model[robot_cost[r]])
def generate_plan ():
    for r in range (R):
        filename = 'robot' + str (r) + '.plan'
        f = open (filename, 'w+')
        for t in range (T):
            coord = str (str (model[X[r][t]]) + " " + str (model[Y[r][t]]) + "\n")
            f.write (coord)
            f.flush ()
        f.close()

generate_plan ()