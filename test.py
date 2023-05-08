from lucky_smt_v3 import main
from math import *
from statistics import *
import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument('-d', dest='dimension', type=int, help='Size of workspace')
parser.add_argument('-r', dest='robot', type=int, help='No of robots')
parser.add_argument('-t', default=1,dest='tests', type=int, help='test no')
#parser.add_argument('-f', dest='filename', type=str, help='Name of the file to save')   
args = parser.parse_args()
D = args.dimension
t = args.tests
R = args.robot

init_loc_file = "INITIAL_LOCATIONS-"+str(1)

tf = 'TESTS_'+str(D)+'x'+str(D)
if not os.path.isdir (tf):
    os.mkdir (tf)

f = ''
if(R!=None):
   f= str(D)+'x'+str(D)+'_'+str(R)+'_test'+str(t)
else:
   f= str(D)+'x'+str(D)+'_test'+str(t)
test_file = os.path.join(tf,f)
f = open (test_file, 'w+')

tests = 10
Dx = D
Dy = D
T = 2 
if(R==None):
    for r in range(4,6):
        R = int(pow(2,r))
        filename = str(D)+'x'+str(D)+'_'+str(R)+'bots'
        if not os.path.isdir (filename):
            os.mkdir (filename)
        K = []
        Time = []
        for i in range(tests):
            print("TEST "+str(i+1))
            test_dir = os.path.join(filename,'TEST'+str(i+1))
            if not os.path.isdir (test_dir):
                os.mkdir (test_dir)
            plots_dir = os.path.join(test_dir, 'plots')
            if not os.path.isdir (plots_dir):
                os.mkdir (plots_dir)

            wp_dir = os.path.join(test_dir, 'WPts')
            if not os.path.isdir (wp_dir):
                os.mkdir (wp_dir)
            path = os.path.join(init_loc_file,str(D)+'x'+str(D)+'_'+str(R),'TEST-'+str(i+1),'robot_init_locs')
            file = open(path,'r')
            init_pos = []
            obst_pos = []
            for r in range(R):
                NewLine = file.readline()
                y,x = int (NewLine.split(' ')[0]), int (NewLine.split(' ')[1])
                init_pos.append((x,y))
            k,total_time =  main(R,T,Dx,Dy,plots_dir,wp_dir,init_pos,obst_pos)
            K.append(k)
            Time.append(total_time)
            if (Time[i]< 60):
                f.write("\nTEST "+str(i+1)+" -------------->   No of horizons : "+str(K[i])+"  ,Computation time : "+str(Time[i])+" sec\n")
                f.flush()
            else:
                f.write("\nTEST "+str(i+1)+" -------------->   No of horizons : "+str(K[i])+"  ,Computation time : "+str(Time[i]/60)+" min\n")
                f.flush()
        
        
        # for i in range(tests):
        #     if (Time[i]< 60):
        #         f.write("\nTEST "+str(i+1)+" -------------->   No of horizons : "+str(K[i])+"  ,Computation time : "+str(Time[i])+" sec\n")
        #         f.flush()
        #     else:
        #         f.write("\nTEST "+str(i+1)+" -------------->   No of horizons : "+str(K[i])+"  ,Computation time : "+str(Time[i]/60)+" min\n")
        #         f.flush()
                
        Avg_k = mean(K)
        Avg_time = mean(Time)
        sd_k = stdev(K)
        sd_time = stdev(Time)
        f.write("\nFor "+str(D)+"x"+str(D)+" grid & "+str(R)+" robots in "+str(tests)+" tests ------>>>> Average no of horizons needed  : "+str(Avg_k)+" , Standard Deviation : "+str(sd_k)+"\n")
        f.flush()
        if (Avg_time< 60):
            f.write("\nFor "+str(D)+"x"+str(D)+" grid & "+str(R)+" robots in "+str(tests)+" tests ------>>>> Average Computation time needed  : "+str(Avg_time)+" sec, Standard Deviation : "+str(sd_time)+" sec\n")
            f.flush()
        else:
            f.write("\nFor "+str(D)+"x"+str(D)+" grid & "+str(R)+" robots in "+str(tests)+" tests ------>>>> Average Computation time needed  : "+str(Avg_time/60)+" min, Standard Deviation : "+str(sd_time)+" sec\n")
            f.flush()
        f.write("\n########################################################################################################################\n")
else:
    filename = str(D)+'x'+str(D)+'_'+str(R)+'bots'
    if not os.path.isdir (filename):
            os.mkdir (filename)
    K = []
    Time = []
    for i in range(tests):
        print("TEST "+str(i+1))
        test_dir = os.path.join(filename,'TEST'+str(i+1))
        if not os.path.isdir (test_dir):
            os.mkdir (test_dir)
        plots_dir = os.path.join(test_dir, 'plots')
        if not os.path.isdir (plots_dir):
            os.mkdir (plots_dir)

        wp_dir = os.path.join(test_dir, 'WPts')
        if not os.path.isdir (wp_dir):
            os.mkdir (wp_dir)
        path = os.path.join(init_loc_file,str(D)+'x'+str(D)+'_'+str(R),'TEST-'+str(i+1),'robot_init_locs')
        file = open(path,'r')
        init_pos = []
        obst_pos = []
        for r in range(R):
            NewLine = file.readline()
            y,x = int (NewLine.split(' ')[0]), int (NewLine.split(' ')[1])
            init_pos.append((x,y))
        k,total_time =  main(R,T,Dx,Dy,plots_dir,wp_dir,init_pos,obst_pos)
        K.append(k)
        Time.append(total_time)
        if (Time[i]< 60):
            f.write("\nTEST "+str(i+1)+" -------------->   No of horizons : "+str(K[i])+"  ,Computation time : "+str(Time[i])+" sec\n")
            f.flush()
        else:
            f.write("\nTEST "+str(i+1)+" -------------->   No of horizons : "+str(K[i])+"  ,Computation time : "+str(Time[i]/60)+" min\n")
            f.flush()
    
    
    # for i in range(tests):
    #     if (Time[i]< 60):
    #         f.write("\nTEST "+str(i+1)+" -------------->   No of horizons : "+str(K[i])+"  ,Computation time : "+str(Time[i])+" sec\n")
    #         f.flush()
    #     else:
    #         f.write("\nTEST "+str(i+1)+" -------------->   No of horizons : "+str(K[i])+"  ,Computation time : "+str(Time[i]/60)+" min\n")
    #         f.flush()
            
    Avg_k = mean(K)
    Avg_time = mean(Time)
    sd_k = stdev(K)
    sd_time = stdev(Time)
    f.write("\nFor "+str(D)+"x"+str(D)+" grid & "+str(R)+" robots in "+str(tests)+" tests ------>>>> Average no of horizons needed  : "+str(Avg_k)+" , Standard Deviation : "+str(sd_k)+"\n")
    f.flush()
    if (Avg_time< 60):
        f.write("\nFor "+str(D)+"x"+str(D)+" grid & "+str(R)+" robots in "+str(tests)+" tests ------>>>> Average Computation time needed  : "+str(Avg_time)+" sec, Standard Deviation : "+str(sd_time)+" sec\n")
        f.flush()
    else:
        f.write("\nFor "+str(D)+"x"+str(D)+" grid & "+str(R)+" robots in "+str(tests)+" tests ------>>>> Average Computation time needed  : "+str(Avg_time/60)+" min, Standard Deviation : "+str(sd_time)+" sec\n")
        f.flush()
    f.write("\n########################################################################################################################\n")