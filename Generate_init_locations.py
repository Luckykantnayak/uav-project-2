import argparse
import os
from random_lib import *
parser = argparse.ArgumentParser()
parser.add_argument('-r', dest='num_robots', type=int, help='Number of robots')
parser.add_argument('-d', dest='dimension', type=int, help='Size of workspace')
parser.add_argument('-t', default=1, dest='tests', type=int, help='Number of tests')
args = parser.parse_args()
R = int(args.num_robots)
D = int(args.dimension)
tests = int(args.tests)

init_loc = "INITIAL_LOCATIONS-3"
if not os.path.isdir (init_loc):
    os.mkdir (init_loc)

GR_dir = os.path.join(init_loc,str(D)+'x'+str(D)+'_'+str(R))
if not os.path.isdir (GR_dir):
    os.mkdir (GR_dir)

for t in range(tests):
    test_dir = os.path.join(GR_dir,"TEST-"+str(t+1))
    if not os.path.isdir (test_dir):
        os.mkdir (test_dir)

    f = 'robot_init_locs'
    test_file = os.path.join(test_dir,f)
    f = open (test_file, 'w+')
    init_bot_pos = get_close_init_positions(D,D,R,obst_pos=[])
    for pos in init_bot_pos:
        (x,y) = pos
        s = str (y) + " " + str (x) + "\n"
        f.write (s)
        f.flush ()
