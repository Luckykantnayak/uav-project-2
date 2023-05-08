import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument('-r', dest='num_robots', type=int, help='Number of robots')
parser.add_argument('-d', dest='dimension', type=int, help='Size of workspace')
parser.add_argument('-t', default=10, dest='tests', type=int, help='Number of tests')


args = parser.parse_args()
D = int(args.dimension)
R = int(args.num_robots)
tests = int(args.tests)

filename = str(D)+'x'+str(D)+'_'+str(R)+'bots'

init_loc_file = "INITIAL_LOCATIONS-"+str(3)

gamrcpp = "GAMRCPP"
if not os.path.isdir (gamrcpp):
    os.mkdir (gamrcpp)

init_dir = os.path.join(gamrcpp,init_loc_file)
if not os.path.isdir (init_dir):
    os.mkdir (init_dir)
GR_dir = os.path.join(init_dir,str(D)+'x'+str(D)+'_'+str(R))
if not os.path.isdir (GR_dir):
    os.mkdir (GR_dir)

for it in range(tests):
    path = os.path.join(init_loc_file,str(D)+'x'+str(D)+'_'+str(R),'TEST-'+str(it+1),'robot_init_locs')
    file = open(path,'r')
    init_pos = []
    for r in range(R):
        NewLine = file.readline()
        y,x = int (NewLine.split(' ')[0]), int (NewLine.split(' ')[1])
        init_pos.append((x,y))

    test_dir = os.path.join(GR_dir,"TEST-"+str(it+1))
    if not os.path.isdir (test_dir):
        os.mkdir (test_dir)
    f = 'robot_ws.txt'
    test_file = os.path.join(test_dir,f)
    f = open (test_file, 'w+')
    for r in range(D):
        s = ""
        for c in range(D):
            pos = (c,r)
            if pos in init_pos:
                idx = init_pos.index(pos)
                s+=str(float(idx+1))+","
            else:
                s+=str(0.5)+","
        s+="\n"
        f.write (s)
        f.flush ()


