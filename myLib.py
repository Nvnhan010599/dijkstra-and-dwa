def Init_S(start):
	return [[start,0,start]]

def check_visited(list_visited,point):
	for i in range(len(list_visited)):
		if(point == list_visited[i]):
			return 1
	return 0

def Init_U(n_vertex):
	inf = 10e9
	u = []
	for i in range(n_vertex):
		u.append([i,inf,-1])
	return u
def resize_S(S):
	n = len(S)
	res_vertex = []
	res_dis = []
	res_pv = []
	for i in range(n):
		res_vertex.append(S[i][0])
		res_dis.append(S[i][1])
		res_pv.append(S[i][2])
	return [res_vertex,res_dis,res_pv]

def pathFinding(S,start,target):
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
def f_value(x1,y1,x2,y2,resolution):
    path = []
    min_x = min(x1,x2)
    idx = int(abs(x2-x1)/resolution) + 1
    if(idx != 1):
        for i in range(idx):
            tmp_x = round(min_x + resolution*i,2)
            tmp_y = round(fx(x1,y1,x2,y2,tmp_x),2)
            path.append([tmp_x,tmp_y])
    else:
        min_y = min(y1,y2)
        idy = int(abs(y2-y1)/resolution) + 1
        for i in range(idy):
            tmp_y = round(min_y + resolution*i,2)
            tmp_x = round(fx(y1,x1,y2,x2,tmp_y),2)
            path.append([tmp_x,tmp_y])
    return path
def fx(x1,y1,x2,y2,x):
    return y1*(x2-x)/(x2-x1) + y2*(x-x1)/(x2-x1)