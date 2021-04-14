import math
import matplotlib.pyplot as plt
class Dijkstra_Algorithm:
    def __init__(self, start_point, target_point,resolution,robot_radius):
        self.inf = 10e6
        self.start_point = start_point
        self.resolution = resolution
        self.target_point = target_point
        self.DT = [[-1,1],[-1,0],[-1,-1],[0,-1],[1,-1],[1,0],[1,1],[0,1]]
        self.min_dis = self.inf
        self.listIndex_check = []
        self.path = self.inf
    def get_map(self,map_fullIndex):
        self.map_fullIndex = map_fullIndex
    def get_map_para(self,lbx,ubx,lby,uby,step_idx,step_idy):
        self.lbx = lbx
        self.ubx = ubx
        self.lby = lby
        self.uby = uby 
        self.step_idx = step_idx
        self.step_idy = step_idy
        self.n_point = self.step_idx*self.step_idy
        ix_start = round((self.start_point[0] - self.lbx)/self.resolution)
        iy_start = round((self.start_point[1] - self.lby)/self.resolution)
        ix_target = round((self.target_point[0] - self.lbx)/self.resolution)
        iy_target = round((self.target_point[1] - self.lby)/self.resolution)
        index_start = ix_start*step_idy + iy_start
        index_target = ix_target*step_idy + iy_target
        self.start_vertex = index_start
        self.target_vertex = index_target
        
        self.S = [[self.start_vertex,0,self.start_vertex]]
        self.step = 0
        self.k = self.S[self.step][0]
        self.init_U(self.k)
    def get_wall(self,bwx,bwy):
        self.bwx = bwx
        self.bwy = bwy
    def init_U(self,k):
        self.U = [[],[],[],[]]
        for i in range(self.step_idx):
            for j in range(self.step_idy):
                if(self.map_fullIndex[i][j] == 0):
                    index = i*self.step_idy + j
                    self.U[0].append(index)
                    self.U[1].append(self.inf)
                    self.U[2].append(-1)
                    self.U[3].append(0)
        index = self.U[0].index(k) # Find k in U
        self.U[0][index] = k             #
        self.U[1][index] = 0
        self.U[2][index] = -1
        self.U[3][index] = 1
    def min_of_U(self):
        U = self.U
        step_idy = self.step_idy
        map_fullIndex = self.map_fullIndex
        min_dis = self.inf
        idmin = -1
        for i in range(len(U[0])):
            ixk = int(U[0][i]/step_idy)
            iyk = int(U[0][i]%step_idy)
            if(U[3][i] != 1 and map_fullIndex[ixk][iyk] == 0):
                if(min_dis > U[1][i]):
                    min_dis = U[1][i]
                    idmin = i   
        return idmin         
    def create_listIndex_check(self,ix0,iy0,xk,yk):
        DT = self.DT
        step_idx = self.step_idx
        step_idy = self.step_idy
        resolution = self.resolution 
        self.listIndex_check =[]
        lbx = self.lbx
        lby = self.lby
        map_fullIndex = self.map_fullIndex
        for i in range(len(DT)):
            ix = ix0 + DT[i][0]
            iy = iy0 + DT[i][1]
            if(ix >= 0 and ix < step_idx):
                if(iy >= 0 and iy < step_idy):
                    if( map_fullIndex[ix][iy] == 0 ):
                        x_o = lbx + ix*resolution
                        y_o = lby + iy*resolution
                        d = math.sqrt((xk - x_o)**2 + (yk-y_o)**2)
                        if(d < 2*resolution):
                            self.listIndex_check.append([ix,iy])
    def calculus_Dis(self,xk,yk):
        listIndex_check = self.listIndex_check
        step_idy = self.step_idy
        resolution = self.resolution
        step = self.step
        S = self.S
        lbx = self.lbx
        lby = self.lby 
        k = self.k
        for i in range(len(listIndex_check)):
            point = self.U[0].index(listIndex_check[i][0]*step_idy + listIndex_check[i][1])
            if(self.U[3][point] != 1):
                ix = listIndex_check[i][0]
                iy = listIndex_check[i][1]
                x_o = lbx + ix*resolution
                y_o = lby + iy*resolution
                d = math.sqrt((xk - x_o)**2 + (yk-y_o)**2)
                if(d + S[step][1] < self.U[1][point]):
                    self.U[1][point] = d + S[step][1]
                    self.U[2][point] = k
    def re_init(self,idmin):
        self.S.append([self.U[0][idmin],round(self.U[1][idmin],3),self.U[2][idmin]])
        self.U[3][idmin] = 1
        self.step = self.step + 1
        self.k = self.S[self.step][0]
    def pathFinding(self,start,target):
        S = self.S
        if(start != S[0][0]):
            print("invalid start vertex")
            return -1
        path = []
        vertex = target
        pv_i = vertex
        while(vertex != start):
            path.append(vertex)
            pv_i = S[0].index(vertex)
            vertex = S[2][pv_i]
        path.append(start)
        path.reverse()
        return path
    def reshape_S(self):
        S = self.S
        n = len(S)
        res_vertex = []
        res_dis = []
        res_pv = []
        for i in range(n):
            res_vertex.append(S[i][0])
            res_dis.append(S[i][1])
            res_pv.append(S[i][2])
        self.S = [res_vertex,res_dis,res_pv]
    def result(self):
        self.reshape_S()
        path = self.pathFinding(self.start_vertex,self.target_vertex)
        print("Done")
        return path
    def plot_Path(self,path):
        plt.plot(self.bwx,self.bwy,'k.',label = "Wall and Obs")
        x_path = []
        y_path = []
        print(len(path))
        for point in range(len(path)):
            ix_path = int(path[point]/self.step_idy)
            iy_path = path[point] - ix_path*self.step_idy

            x_path.append(self.lbx + ix_path*self.resolution)
            y_path.append(self.lby + iy_path*self.resolution)

        plt.plot(x_path,y_path,"c",label = "Path")
        plt.plot(self.start_point[0],self.start_point[1],"bD",label = "Start")
        plt.plot(self.target_point[0],self.target_point[1],"rD",label = "Target")        
        plt.legend()

        
        
    


