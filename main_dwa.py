import dwa_library as dl 
from dwa_library import Robot_parameter, DWA
import myLib as ml
import math
import matplotlib.pyplot as plt

Start = [ 3, 5, 0, 0, 0 ]
Target = ml.f_value(3,5,10,5,0.1)
obstacle = [[7,5],[7,4.55],[7.5,5]]

def main_dw(Start,Target,obstacle):
    index = 0
    t = 0
    robot = Robot_parameter()
    dwa_parameter = DWA()
    robot.set_Obstacle(obstacle)
    u_control = [[],[],[]]
    trajectory = []
    X = Start[:]
    while(1):
#-Init--#--------------------------------------------------------------------#
        v = X[3]
        omega = X[4]
        t_i = index
        while(t_i < len(Target) - 1):
            dist = math.hypot(X[0] - Target[t_i][0], X[1] - Target[t_i][1])
            if(dist > robot.robot_radius):
                break
            else :
                t_i += 1
                index = t_i
        Xf = Target[index]
#--DWA--#--------------------------------------------------------------------#
        u, predict_trajectory = dl.dwa(v,omega,Xf,X,robot,dwa_parameter)
#COLECT-#-----------------------------------------------------------------------#
        t += robot.dT
        u_control[0].append(u[0]) # x
        u_control[1].append(u[1]) # y
        u_control[2].append(t)    # t
        trajectory.append(X)      # path
        X = dl.motion_robot(X,u,robot) # dT = 0.2s Get feetback signal through encoder
#-PLOT--#--------------------------------------------------------------------#
        dl.plot_function(Start,Target,predict_trajectory,X,robot)
        plt.plot(Xf[0],Xf[1],"r*")
        plt.pause(0.0001)
#-CHANGE-TARGET--------------------------------------------------------------#
        dist_to_goal = math.hypot(X[0] - Xf[0], X[1] - Xf[1])
        state_X = predict_trajectory[-1][0]
        state_Y = predict_trajectory[-1][1]
        dist_to_predict = math.hypot(state_X - Xf[0], state_Y - Xf[1])
        if( dist_to_predict <= robot.error_mode and index < len(Target) - 1):
            index = index + 1
            Xf = Target[index] 
            Target = dl.do_obstacle(Xf,robot,Target,index)
            Xf = Target[index]
            
#-STOP-CONDITION--------------------------------------------------------------#
        if dist_to_goal <= robot.error_standard and index == len(Target) - 1 :
            print("Goal!!")
            break 

    for point in trajectory:
        plt.plot(point[0],point[1],"r.")
    print(X)
    plt.axis("equal")
    plt.grid(True)

    plt.figure()
    plt.plot(u_control[2],u_control[0],'r',label = "V")
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(u_control[2],u_control[1],'b',label = "omega")
    plt.legend()
    plt.grid(True)

    plt.show()       

main_dw(Start,Target,obstacle)
