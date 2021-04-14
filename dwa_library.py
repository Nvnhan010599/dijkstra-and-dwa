import math
import matplotlib.pyplot as plt
class Robot_parameter:
    def __init__(self):
        self.v_min = -0.5
        self.v_max = 0.5
        self.av = 0.2
        self.resol_v = 0.01
        self.omega_max = 60.0*math.pi/180.0
        self.omega_min = - self.omega_max
        self.aw = 30*math.pi/180.0
        self.resol_w = 0.1*math.pi/180.0
        self.robot_radius = 0.5
        self.robot_stuck_flag_cons = 0.01
        self.error_mode =  self.robot_radius/5.0
        self.error_mode_predict = self.robot_radius
        self.error_standard = self.robot_radius/10
        self.dT = 0.2
        self.dT_predict = 2

        self.inf = 10e6
        self.obstacle = []
    def set_Obstacle(self,obstacle):
        self.obstacle = obstacle

class DWA(Robot_parameter):
    def __init__(self):
        self.alpha = 0.5
        self.peta = 0.3
        self.gamma = 1

def heading(Xf,traject_predicted):
    x_n = traject_predicted[-1][0]
    y_n = traject_predicted[-1][1]
    theta_n = traject_predicted[-1][2]
    dx = Xf[0] - x_n
    dy = Xf[1] - y_n
    e_a = math.atan2( dy,dx  )
    c_a = e_a - theta_n
    return abs(math.atan2( math.sin(c_a), math.cos(c_a) ) )
def dis(traject_predicted,robot):
    list_r = []
    for pre_pos in traject_predicted:
        x_p = pre_pos[0]
        y_p = pre_pos[1]
        for ob in robot.obstacle:
            x_o = ob[0]
            y_o = ob[1]
            distance = math.sqrt((x_p - x_o)**2 + (y_p - y_o)**2)
            if(distance <= robot.robot_radius):
                return robot.inf
            list_r.append(distance)
    if(len(list_r) != 0):
        min_dis = min(list_r)
        return 1.0/min_dis
    else:
        return 0
def motion_robot(tx,u,robot):
    x = tx[:]
    x[2] = round(x[2] + u[1]*robot.dT,4)
    x[0] = round(x[0] + u[0]*math.cos(x[2])*robot.dT,4)
    x[1] = round(x[1] + u[0]*math.sin(x[2])*robot.dT,4)
    x[3] = u[0]
    x[4] = u[1]
    return x 
def predict_motion(x_init,u,robot):
    traject_predicted = []
    time = 0
    tmp_X = x_init[:]
    while(time <= robot.dT_predict):
        tmp_X = motion_robot(tmp_X,u,robot)
        traject_predicted.append(tmp_X)
        time += robot.dT
    return traject_predicted
def dwa(v,omega,Xf,X,robot,dwa_para):
    lbv = max(robot.v_min, v - robot.av*robot.dT)
    ubv = min(robot.v_max, v + robot.av*robot.dT)
    lbw = max(robot.omega_min, omega - robot.aw*robot.dT )
    ubw = min(robot.omega_max, omega + robot.aw*robot.dT )
    idv = round((ubv - lbv)/robot.resol_v) + 1
    idw = round((ubw - lbw)/robot.resol_w) + 1
    G = 0
    min_G = robot.inf
    best_u = [0,0]
    best_trajectory = [[],[],[]]
    for i in range(idv):
        vd = lbv + i*robot.resol_v
        for j in range(idw):
            wd = lbw + j*robot.resol_w
            u = [vd,wd]#----------------Model_Predict-------------#
            traject_predicted = predict_motion(X,u,robot)
            #--------------------G(vd,wd)--------------------#
            heading_g = heading(Xf,traject_predicted)
            dis_g = dis(traject_predicted,robot)
            vel_g = robot.v_max - traject_predicted[-1][3]
            G = dwa_para.alpha*heading_g + dwa_para.peta*dis_g + dwa_para.gamma*vel_g
            if(min_G > G):
                best_u = [vd,wd]
                min_G = G
                if(abs(vd) <= robot.robot_stuck_flag_cons and abs(X[3]) < robot.robot_stuck_flag_cons):
                    best_u[1] = robot.omega_min
                best_trajectory = traject_predicted
    return best_u, best_trajectory
def plot_function(Start,Target,predict_trajectory,X,robot):
    plt.cla()
    plt.plot(Start[0],Start[1],"*r")
    plt.plot(X[0],X[1],'ro')
    for i in range(len(Target)):
        plt.plot(Target[i][0],Target[i][1],".b")
    for ob in robot.obstacle:
        plt.plot(ob[0],ob[1],"ko")
        circle = plt.Circle((ob[0], ob[1]), robot.robot_radius, color="k",fill=False)
        plt.gcf().gca().add_artist(circle)
    for point in predict_trajectory:
        plt.plot(point[0],point[1],"g.")
    plt.pause(0.0001)

def do_obstacle(Xf,robot,tTarget,index):
    Target = tTarget.copy()
    tmp_index = index
    i = 0
    ob = robot.obstacle[i]
    dist_to_goal = math.hypot(ob[0] - Xf[0], ob[1] - Xf[1])
    if( dist_to_goal <= robot.robot_radius*2):
        if(i == 2):
            pass
        tmp_index = index + 1
        while(1):
            ob = robot.obstacle[i]
            X_tmp = Target[tmp_index]
            dist_to_goal1 = math.hypot(ob[0] - X_tmp[0], ob[1] - X_tmp[1])
            if( dist_to_goal1 <= robot.robot_radius*1.2):
                del Target[tmp_index]
            else: 
                i += 1
            if(i > len(robot.obstacle) - 1):
                break

    return Target

