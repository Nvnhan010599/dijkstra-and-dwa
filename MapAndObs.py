import matplotlib.pyplot as plt
import math

Visualize = False

wx = [0,0,3,7,7,4,4,2,2]
wy = [0,6,6,5,0,0,3,2,0]

obstackle_x = [1,2,5.5,5.5]
obstackle_y = [4,4,2,4]

ox = []
oy = []

resolution = 0.2
robot_radius = 0.5

inf = 1e6

def f_value(x1,y1,x2,y2,resolution):
    x = []
    y = []
    min_x = min(x1,x2)
    idx = int(abs(x2-x1)/resolution) + 1
    if(idx != 1):
        for i in range(idx):
            tmp_x = round(min_x + resolution*i,2)
            tmp_y = round(fx(x1,y1,x2,y2,tmp_x),2)
            x.append(tmp_x)
            y.append(tmp_y)
    else:
        min_y = min(y1,y2)
        idy = int(abs(y2-y1)/resolution) + 1
        for i in range(idy):
            tmp_y = round(min_y + resolution*i,2)
            tmp_x = round(fx(y1,x1,y2,x2,tmp_y),2)
            y.append(tmp_y)
            x.append(tmp_x)
    return x,y

def fx(x1,y1,x2,y2,x):
    return y1*(x2-x)/(x2-x1) + y2*(x-x1)/(x2-x1)


bwx = []
bwy = []
listOfX = []
# Tao diem tren tuong
for i in range(len(wx)):
    x2 = wx[i]
    y2 = wy[i]
    x1 = wx[i-1]
    y1 = wy[i-1]
    x , y = f_value(x1,y1,x2,y2,resolution)
    bwx += x
    bwy += y
# Tao diem obstackle
for i in range(1,len(obstackle_x),2):
    x2 = obstackle_x[i]
    y2 = obstackle_y[i]
    x1 = obstackle_x[i-1]
    y1 = obstackle_y[i-1]
    tx , ty = f_value(x1,y1,x2,y2,resolution)
    ox += tx
    oy += ty

lbx = min(wx)
ubx = max(wx)
lby = min(wy)
uby = max(wy)
step_idx = int((ubx - lbx)/resolution) + 1
step_idy = int((uby - lby)/resolution) + 1

limit_x = []


# Gioi Han cac diem ben trong Map
for i in range(step_idx):
    # Sap xep theo chieu tang cua X va gioi han Y tai X
    limit_x += [[round(lbx + i*resolution,2),inf,-inf]] # x = lbx + i*resolution

for i in range(len(bwx)):
    index = round((bwx[i]-lbx)/resolution)
    if(limit_x[index][1] > bwy[i] ):
        limit_x[index][1] = bwy[i]
    if(limit_x[index][2] < bwy[i]):
        limit_x[index][2] = bwy[i]
# Lap day cac diem trong Map


map_fullIndex = []
map_checkIndex = []
for i in range(step_idx):
    tmp_map = []
    tmp_map_c = []
    for j in range(step_idy):
        state = -1
        tmp_y = lby + j*resolution
        if(tmp_y >= limit_x[i][1] and tmp_y <= limit_x[i][2]):
            state = 0
        tmp_map.append(state)
        tmp_map_c.append(0)
    map_fullIndex.append(tmp_map)
    map_checkIndex.append(tmp_map_c)

bwx += ox # Noi tuong voi vat chan 
bwy += oy #

Do = []
range_radius = round(robot_radius/resolution) 
for i in range(-range_radius,range_radius+1):
    for j in range(-range_radius,range_radius+1):
        Do.append([i,j])



for p in range(len(bwx)):
    t_x = bwx[p]
    t_y = bwy[p]

    ix0 = round((t_x - lbx)/resolution)
    iy0 = round((t_y - lby)/resolution)
    for i in range(len(Do)):
        ix = ix0 + Do[i][0]
        iy = iy0 + Do[i][1]
        if(ix >= 0 and ix < step_idx):
            if(iy >= 0 and iy < step_idy):
                if(map_fullIndex[ix][iy] == 0):
                    x_o = lbx + ix*resolution
                    y_o = lby + iy*resolution

                    d = math.sqrt((t_x - x_o)**2 + (t_y-y_o)**2)
                    if(d <= robot_radius):
                        map_fullIndex[ix][iy] = -1

if Visualize :
    plt.plot(bwx,bwy,'ko') 
for i in range(step_idx):
    for j in range(step_idy):
        t_x = lbx + i*resolution
        t_y = lby + j*resolution
        if(map_fullIndex[i][j] == 0):
            #plt.plot(t_x,t_y,'c.')
            pass
        if(map_fullIndex[i][j] == -1):
            #plt.plot(t_x,t_y,'r,')
            pass
    """
    result : 
        lbx ubx
        lby uby
        step_idx = int((ubx - lbx)/resolution) + 1
        step_idy = int((uby - lby)/resolution) + 1

        resolution
        robot_radius

        map_fullIndex

    """
#plt.show()
