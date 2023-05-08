import argparse
import os
home_directory = os.path.expanduser( '~' )
output_dir = os.path.join( home_directory, 'gamrcpp_ws', 'src','GAMRCPP','output' )
parser = argparse.ArgumentParser()
parser.add_argument('-r', dest='num_robots', type=int, help='Number of robots')
parser.add_argument('-d', dest='dimension', type=int, help='Size of workspace')
parser.add_argument('-t', default=10, dest='tests', type=int, help='Number of tests')


args = parser.parse_args()
D = int(args.dimension)
R = int(args.num_robots)
tests = int(args.tests)

filename = str(D)+'x'+str(D)+'_'+str(R)+'bots'

init_loc_file = "INITIAL_LOCATIONS-"+str(1)

hrz = 0

while(1):
    file = "cap_"+hrz+str(".csv")
    