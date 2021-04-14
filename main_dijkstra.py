from MapAndObs import map_fullIndex
from MapAndObs import lbx, ubx, lby, uby
from MapAndObs import step_idx, step_idy # 71, 61
from MapAndObs import resolution, robot_radius
from MapAndObs import bwx,bwy
import myLib
import math
import matplotlib.pyplot as plt
from dijkstra_library import Dijkstra_Algorithm

def main_dijkstra(Start,Target):
    dijkstra = Dijkstra_Algorithm(Start,Target,resolution,robot_radius)
    dijkstra.get_map(map_fullIndex)
    dijkstra.get_map_para(lbx,ubx,lby,uby,step_idx,step_idy)
    dijkstra.get_wall(bwx,bwy)
    
    while(dijkstra.k != dijkstra.target_vertex):
        k = dijkstra.k
        ix0 = int(k/step_idy)
        iy0 = int(k%step_idy)
        xk = lbx + ix0*resolution
        yk = lby + iy0*resolution
        dijkstra.create_listIndex_check(ix0,iy0,xk,yk)
        dijkstra.calculus_Dis(xk,yk)
        idmin = dijkstra.min_of_U()
        dijkstra.re_init(idmin)
    path = dijkstra.result()
    print(path)
    dijkstra.plot_Path(path)
    plt.show()
start_point = [1,1] # input
target_point = [5.6,1] # input
main_dijkstra(start_point,target_point)
