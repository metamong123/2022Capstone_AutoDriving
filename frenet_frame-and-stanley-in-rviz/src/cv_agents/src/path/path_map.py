#!/usr/bin/python
#-*- coding: utf-8 -*-

import pickle
import sys
import rospkg
import numpy as np
from scipy.interpolate import interp1d

rospack = rospkg.RosPack()
path_map = rospack.get_path("map_server")
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/")
sys.path.append(path_map + "/src/")

class MakingPath:
	def __init__(self):
		self.x = []
		self.y = []
		self.yaw = []

def interpolate_waypoints(wx, wy, space=0.5):
	_s = 0
	s = [0]
	for i in range(1, len(wx)):
		prev_x = wx[i - 1]
		prev_y = wy[i - 1]
		x = wx[i]
		y = wy[i]

		dx = x - prev_x
		dy = y - prev_y

		_s = np.hypot(dx, dy)
		s.append(s[-1] + _s)

	fx = interp1d(s, wx)
	fy = interp1d(s, wy)
	ss = np.linspace(0, s[-1], num=int(s[-1] / space) + 1, endpoint=True)

	dxds = np.gradient(fx(ss), ss, edge_order=1)
	dyds = np.gradient(fy(ss), ss, edge_order=1)
	wyaw = np.arctan2(dyds, dxds)

	return {
		"x": fx(ss),
		"y": fy(ss),
		"yaw": wyaw,
		"s": ss
	}

class Path:
	def __init__(self,pc_route):
		self.nodes={}
		self.nodes['global']={}
		self.nodes['horizontal_parking']={}
		self.nodes['diagonal_parking']={}
		self.nodes['delivery']={}
		self.global_route=pc_route #global pkl파일 경로
		with open(pc_route, 'rb') as f:
			self.nodes['global']=pickle.load(f)
		self.horizontal_parking_route=[] #parking pkl파일 경로
		self.diagonal_parking_route=[]
		self.delivery_route=[] #delivery pkl파일 경로
		self.horizontal_parking_path={}
		self.diagonal_parking_path={}
		self.delivery_path={}
		self.w={}
		self.horizontal_parking_map_num=0 # 주차 구역 수
		self.diagonal_parking_map_num=0 # 주차 구역 수
		self.delivery_map_num=0 # 배달 구역 수
		self.waypoints={}
		self.link_len={}
		self.link_dir={'straight':[],'left':[],'right':[]}
		self.target_speed={}
		self.stopline_start_list=[] #stopline range 시작점
		self.stopline_finish_list=[] #stopline camera 인식 마지노선
		# 예선 구간의 경우에는 저속 관련 변수를 int로 사용해야할 수도 있음
		self.low_speed_start_list=[] #curve, 어린이 보호 구역 등 저속 range 시작점
		self.low_speed_finish_list=[] # 저속 range 끝 점
		self.glo_to_dynamic_start=0 # 동적 장애물 구간 시작점
		self.glo_to_dynamic_finish=0 # 동적 장애물 구간 끝 점
		self.glo_to_horizontal_park_start=0 # global->horizontal_parking 시작 waypoint
		self.glo_to_horizontal_park_finish=0 # global->horizontal_parking 시작 waypoint range 설정
		self.glo_to_diagonal_park_start=0 # global->diagonal_parking 시작 waypoint
		self.glo_to_diagonal_park_finish=0 # global->diagonal_parking 시작 waypoint range 설
		self.glo_to_del_start=[]
		self.glo_to_del_finish=[]
		self.horizontal_parking_stop=[] # 주차 구역마다 주차 정지 waypoint 설정
		self.horizontal_park_to_glo=[] # parking->global 변경 waypoint 지점
		self.diagonal_parking_stop=[] # 주차 구역마다 주차 정지 waypoint 설정
		self.diagonal_park_to_glo=[] # parking->global 변경 waypoint 지점
		self.lane_width={} # example lane_width={'left':3.3(우리 차선 width):2.2(왼쪽으로 갈 수 있는 width):0, 'right':3.3:2.2:1, 'none':3.3:2}        {'3.3':[0],'3.8':[1],'4.1':[2], '6.6':[3]...}
		self.DF_SET={}
	def set_other_mode(self, mode='parking', pc_route=path_map+"/src/frontier/parking_route.pkl", link=None):
		if not link==None:
			self.nodes[mode][link]={}
			if mode=='diagonal_parking':
				with open(pc_route, 'rb') as f:
					file=pickle.load(f)
					self.nodes[mode][link]=file[0]
					self.nodes[mode][link+1]={}
				for i in self.nodes[mode][link].keys():
					self.nodes[mode][link+1][i]=list(reversed(self.nodes[mode][link][i]))
			elif mode == 'horizontal_parking':
				with open(pc_route, 'rb') as f:
					file=pickle.load(f)
					self.nodes[mode][link+1]={}
					self.nodes[mode][link+1]=file[0]
				for i in self.nodes[mode][link+1].keys():
					self.nodes[mode][link][i]=list(reversed(self.nodes[mode][link+1][i]))
			else:
				for i in range(self.delivery_map_num):
					with open(pc_route, 'rb') as f:
						file=pickle.load(f)
						self.nodes[mode][link]=file[0]		
		else:
			self.nodes[mode][0]={}
			if mode=='diagonal_parking':
				with open(pc_route, 'rb') as f:
					file=pickle.load(f)
					self.nodes[mode][0]=file[0]
					self.nodes[mode][link+1]={}
				for i in self.nodes[mode][0].keys():
					self.nodes[mode][1][i]=list(reversed(self.nodes[mode][0][i]))
			elif mode == 'horizontal_parking':
				with open(pc_route, 'rb') as f:
					file=pickle.load(f)
					self.nodes[mode][1]={}
					self.nodes[mode][1]=file[0]
				for i in self.nodes[mode][0].keys():
					self.nodes[mode][0][i]=list(reversed(self.nodes[mode][1][i]))
			else:
				with open(pc_route, 'rb') as f:
					file=pickle.load(f)
					self.nodes[mode][0]=file[0]

	def set_link(self, waypoint_list=[0],mode='global'):
		
		nodes_len=len(self.nodes[mode][0]['x'])
		waypoint_list.append(nodes_len)
		
		for i in reversed(range(len(waypoint_list)-1)):
			self.nodes[mode][i]={'x':self.nodes[mode][0]['x'][waypoint_list[i]:waypoint_list[i+1]], 'y':self.nodes[mode][0]['y'][waypoint_list[i]:waypoint_list[i+1]], 's':self.nodes[mode][0]['s'][waypoint_list[i]:waypoint_list[i+1]], 'yaw':self.nodes[mode][0]['yaw'][waypoint_list[i]:waypoint_list[i+1]]}

	def set_dir(self, straight, left, right):
		self.link_dir={'straight':straight,'left':left,'right':right}
	
	def set_waypoint_range(self, waypoint_start_list=None, waypoint_finish_list=None):
		w_f_l=waypoint_finish_list
	
		if waypoint_start_list == None:
			w_s_l=[sw-8 for sw in waypoint_finish_list] # for stopline
		else:
			w_s_l=waypoint_start_list
	
		return w_s_l, w_f_l

	def set_map(self, del_space=0.5, glo_space=0.5, park_space=0.5):
		for i in self.nodes.keys():
			self.w[i]={}
			self.waypoints[i]={}
			self.link_len[i]=[]

			if i == 'global':
				self.w[i]={'x':[],'y':[]}
				self.waypoints[i]={'x':[],'y':[],'yaw':[],'s':[]}
			
			for j in range(len(self.nodes[i].keys())):
				if i == 'horizontal_parking' or i =='diagonal_parking':
					if j %2==0:
						self.w[i][j]={'x':[],'y':[]}
						self.waypoints[i][j]={'x':[],'y':[],'yaw':[],'s':[]}
				elif i == 'delivery':
					self.w[i][j]={'x':[],'y':[]}
					self.waypoints[i][j]={'x':[],'y':[],'yaw':[],'s':[]}

				for k in ('x','y'):
					if i == 'global':
						self.w[i][k].append(self.nodes[i][j][k][1:])
					elif i == 'horizontal_parking' or i =='diagonal_parking':
						if j%2==0:
							self.w[i][j][k].append(self.nodes[i][j][k][1:])
						else:
							self.w[i][j-1][k].append(self.nodes[i][j][k][1:])
					elif i == 'delivery':
						self.w[i][j][k].append(self.nodes[i][j][k][1:])

			for k in ('x','y'):
				if i == 'global':
					self.w[i][k] = np.concatenate(self.w[i][k])
			
			for j in range(len(self.nodes[i].keys())):
				for k in ('x','y'):
					if i == 'horizontal_parking' or i =='diagonal_parking':
						if j%2==0:
							self.w[i][j][k] = np.concatenate(self.w[i][j][k])
					elif i =='delivery':
						self.w[i][j][k] = np.concatenate(self.w[i][j][k])

			if i == 'global':
				self.waypoints[i] = interpolate_waypoints(self.w[i]['x'], self.w[i]['y'], space=glo_space)
			elif i == 'horizontal_parking' or i =='diagonal_parking':
				for j in range(len(self.nodes[i].keys())):
					if j%2==0:
						self.waypoints[i][j] = interpolate_waypoints(self.w[i][j]['x'], self.w[i][j]['y'], space=park_space)
			elif i=='delivery':
				for j in range(len(self.nodes[i].keys())):
					self.waypoints[i][j] = interpolate_waypoints(self.w[i][j]['x'], self.w[i][j]['y'],space=del_space)

			link_i=-1
			for j in range(len(self.nodes[i].keys())):
				if i == 'horizontal_parking' or i =='diagonal_parking':
					if j % 2 ==0:
						link_i=-1
						link_i+=len(self.nodes[i][j]["x"])
						self.link_len[i].append(link_i)
					else:
						link_i+=len(self.nodes[i][j]["x"])
						self.link_len[i].append(link_i)
				elif i=='delivery':
					link_i=-1
					link_i+=len(self.nodes[i][j]["x"])
					self.link_len[i].append(link_i)
				else:
					link_i+=len(self.nodes[i][j]["x"])
					self.link_len[i].append(link_i)
	def make_path(self, mode, map_num):
		path={}
		if mode == 'horizontal_parking':
			for link_ind in range(0,map_num*2,2):
				way=MakingPath()
				way.x=self.waypoints[mode][link_ind]['x'][:self.link_len[mode][link_ind]]
				way.y=self.waypoints[mode][link_ind]['y'][:self.link_len[mode][link_ind]]
				way.yaw=self.waypoints[mode][link_ind]['yaw'][:self.link_len[mode][link_ind]]
				path[link_ind]=[way.x, way.y, way.yaw] #짝수 후진 홀수 전진
		if mode == 'diagonal_parking':
			for link_ind in range(0,map_num*2,2):
				way=MakingPath()
				way.x=self.waypoints[mode][link_ind]['x'][:self.link_len[mode][link_ind]]
				way.y=self.waypoints[mode][link_ind]['y'][:self.link_len[mode][link_ind]]
				way.yaw=self.waypoints[mode][link_ind]['yaw'][:self.link_len[mode][link_ind]]
				path[link_ind/2]=[way.x, way.y, way.yaw] #짝수 전진 (홀수 후진)
		else:
			for link_ind in range(map_num):
				way=MakingPath()
				way.x=self.waypoints[mode][link_ind]['x'][:self.link_len[mode][link_ind]]
				way.y=self.waypoints[mode][link_ind]['y'][:self.link_len[mode][link_ind]]
				way.yaw=self.waypoints[mode][link_ind]['yaw'][:self.link_len[mode][link_ind]]
				path[link_ind]=[way.x, way.y, way.yaw]
		return path
	
	def set_lanewidth(self):
		for i in self.lane_width.keys(): #left, light, none
			if i == 'left':
				for j in self.lane_width[i]: # j=lane_width
					LANE_WIDTH = j
					for k in self.lane_width[i][j]: # k=left_width, left_width+lanewidth/2 ~ lanewidth/2
						width = k
						for l in self.lane_width[i][j][k]: # l = link_index
							self.DF_SET[l]=np.array([0, LANE_WIDTH/2, -LANE_WIDTH/2, width])
			elif i =='right':
				for j in self.lane_width[i]: # j=lane_width
					LANE_WIDTH = j
					for k in self.lane_width[i][j]: # k=left_width, left_width+lanewidth/2 ~ lanewidth/2
						width = k
						for l in self.lane_width[i][j][k]: # l = link_index
							self.DF_SET[l]=np.array([0, LANE_WIDTH/2, -LANE_WIDTH/2, -width])
			elif i =='none':
				for j in self.lane_width[i]: # j=lane_width
					LANE_WIDTH = j
					for l in self.lane_width[i][j]: # l = link_index
						self.DF_SET[l]=np.array([0, LANE_WIDTH/2, -LANE_WIDTH/2])


	# def set_lanewidth(self, link_ind, dir=None, width=None):
	# 	for i in self.lane_width.keys():
	# 		for j in self.lane_width[i]:
	# 			if link_ind == j:
	# 				LANE_WIDTH=i
	# 	if dir=='left':
	# 		self.DF_SET=np.array([0, LANE_WIDTH/2, -LANE_WIDTH/2, width])
	# 	elif dir == 'right':
	# 		self.DF_SET=np.array([0, LANE_WIDTH/2, -LANE_WIDTH/2, -width])
	# 	elif dir==None:
	# 		self.DF_SET=np.array([0, LANE_WIDTH/2, -LANE_WIDTH/2])

def frontier():
	frontier=Path(path_map + "/src/frontier/curve_test.pkl")
	frontier.set_link([0,100,200,300,400,500,600,700,800,900,1000,1100])
	frontier.set_dir([0,1],[2],[3,4,5,6,7,8,9,10,11,12,13])
	frontier.target_speed={'global':{'straight':20/3.6, 'curve':20/3.6},'parking':8/3.6,'delivery':10/3.6}
	frontier.set_map()
	frontier.lane_width={'none':{3.0:[i for i in range(14)]}}
	frontier.set_lanewidth()
	return frontier

def kcity():
	kcity=Path(path_map + "/src/kcity/route.pkl")
	kcity.set_link([0,110,190,280,390,530,600,670,720,780,880,940,970,1010,1190,1220,1260,1300,1360,1450,1580,1600,1640,1730,1780,1850,1910,2060,2140,2190,2260,2400,2510,2670,2760,2880,2960,3138])
	kcity.set_dir([0,1,2,3,4,6,7,8,9,11,13,15,17,18,19,20,22,24,26,30,31,33,34,35,36,37],[21,23,25,27],[5,10,12,14,16,28,29,32])    
	kcity.stopline_start_list,kcity.stopline_finish_list=kcity.set_waypoint_range(waypoint_finish_list=[190,280,390,670,880,970,1260,1450,1730,2060,2190,2400,2670,2760,2880])
	
	kcity.diagonal_parking_map_num=6
	park_ver="v1"
	for i in range(kcity.diagonal_parking_map_num):
		park_route=path_map+"/src/kcity/diagonal_parking_"+park_ver+"_"+str(i)+".pkl"
		kcity.diagonal_parking_route.append(park_route)
		kcity.set_other_mode(mode='diagonal_parking', pc_route=park_route,link=2*i)

	kcity.delivery_map_num=2
	for i in range(kcity.delivery_map_num):
		del_route=path_map+"/src/kcity/delivery_"+str(i)+".pkl"
		kcity.delivery_route.append(del_route)
		kcity.set_other_mode(mode='delivery', pc_route=del_route,link=i)

	kcity.glo_to_diagonal_park_start=110
	kcity.glo_to_diagonal_park_finish=120
	# kcity.diagonal_parking_stop=[]
	# kcity.park_to_glo_start=[]
	# kcity.park_to_glo_finish=[]
	kcity.glo_to_del_start=[720, 1360]
	kcity.glo_to_del_finish=[730, 1370]
	
	kcity.target_speed={'global':20/3.6,'parking':10/3.6, 'delivery':10/3.6}
	kcity.lane_width={'3.3':[4,7,8,9,10,15,16,17,18,19,20,21,22,23,24,25,27,29,30,31,32],'3.8':[0,1,2,3,33,34,35,36],'4.1':[11,12,28], '6.6':[13,14,26], '7.1':[5,6]}
	kcity.set_map()
	kcity.diagonal_parking_path=kcity.make_path('diagonal_parking',kcity.diagonal_parking_map_num)
	kcity.delivery_path=kcity.make_path('delivery',kcity.delivery_map_num)
	return kcity

def boong():
	offset_state = "_offset"
	# offset_state : "", "_offset", "_offset2", "_offset3"
	# old maps : "_old", "_old_offset", "_old_offset2"

	boong=Path(path_map + "/src/boong/global"+offset_state+".pkl")
	
	if offset_state == "_old_offset2":
		boong.set_link([0,20,190,220,420,460,620,680,800,830])
	else:
		boong.set_link([0,20,190,220,420,460,620,680,800,838])

	boong.set_dir([0,1,3,5,7,9,10],[],[2,4,6,8])
	
	boong.diagonal_parking_map_num=9
	for i in range(boong.diagonal_parking_map_num):
		park_route=path_map+"/src/boong/parking"+offset_state+"_"+str(i)+".pkl"
		boong.diagonal_parking_route.append(park_route)
		boong.set_other_mode(mode='diagonal_parking', pc_route=park_route,link=2*i)
	
	# boong.delivery_map_num=2
	# for i in range(boong.delivery_map_num):
	# 	del_route=path_map+"/src/boong/delivery_"+str(i)+".pkl"
	# 	boong.delivery_route.append(del_route)
	# 	boong.set_other_mode(mode='delivery', pc_route=del_route,link=i)	
	
	boong.glo_to_diagonal_park_start=15
	boong.glo_to_diagonal_park_finish=20
	# boong.parking_stop=[]
	# boong.park_to_glo_start=[]
	# boong.park_to_glo_finish=[]
	# boong.glo_to_del_start=[]
	# boong.glo_to_del_finish=[]
	
	boong.target_speed={'global':{'straight':20/3.6, 'curve':12/3.6},'parking':8/3.6,'delivery':10/3.6}
	boong.set_map()
	boong.diagonal_parking_path=boong.make_path('diagonal_parking',boong.diagonal_parking_map_num)
	# boong.delivery_path=boong.make_path('delivery',boong.delivery_map_num)

	boong.lane_width={'none':{3.0:[i for i in range(11)]}}
	boong.set_lanewidth()
	return boong

def delivery_test():
	delivery_test=Path(path_map + "/src/delivery_test/global.pkl")
	delivery_test.set_link([0,380,430,660,740,1080,1180,1400,1487])
	delivery_test.set_dir([0,2,4,6,8,9],[1,3,5,7],[])
	delivery_test.diagonal_parking_map_num=0
	delivery_test.delivery_map_num=2
	for i in range(delivery_test.delivery_map_num):
		del_route=path_map+"/src/delivery_test/delivery_"+str(i)+".pkl"
		delivery_test.delivery_route.append(del_route)
		delivery_test.set_other_mode(mode='delivery', pc_route=del_route,link=i)	
	
	delivery_test.glo_to_del_start=[60, 210]
	delivery_test.glo_to_del_finish=[80, 230]
	delivery_test.glo_to_dynamic_start=800
	delivery_test.glo_to_dynamic_finish=1000
	
	delivery_test.target_speed={'global':{'straight':10/3.6, 'curve':8/3.6},'parking':8/3.6,'delivery':4/3.6}
	delivery_test.set_map()
	delivery_test.delivery_path=delivery_test.make_path('delivery',delivery_test.delivery_map_num)

	delivery_test.lane_width={'none':{3.0:[i for i in range(10)]}}
	delivery_test.set_lanewidth()
	return delivery_test

def delivery_test_cw():
	del_space = 0.5
	glo_space=0.5
	# 0.5 (original) / 0.25 (x2) / 0.125 (x4) / 0.1 (x5)
	delivery_test_cw=Path(path_map + "/src/delivery_test_cw/global_"+str(glo_space)+".pkl")
	link_list=list(map(int,list(np.array([0,180,210,340,360,540,570,690,717])*float(0.5/1))))
	delivery_test_cw.set_link(link_list)
	delivery_test_cw.set_dir([0,2,4,6,8],[],[1,3,5,7])
	
	delivery_test_cw.delivery_map_num=2
	for i in range(delivery_test_cw.delivery_map_num):
		del_route=path_map+"/src/delivery_test_cw/delivery_"+str(i)+"_"+str(del_space)+".pkl"
		delivery_test_cw.delivery_route.append(del_route)
		delivery_test_cw.set_other_mode(mode='delivery', pc_route=del_route,link=i)	
	
	delivery_test_cw.glo_to_del_start=list(np.array([30, 110])*float(0.5/1))
	delivery_test_cw.glo_to_del_finish=list(np.array([40, 120])*float(0.5/1))
	delivery_test_cw.glo_to_dynamic_start=400*float(0.5/1)
	delivery_test_cw.glo_to_dynamic_finish=500*float(0.5/1)
	
	delivery_test_cw.target_speed={'global':{'straight':10/3.6, 'curve':8/3.6},'parking':8/3.6,'delivery':4/3.6}
	delivery_test_cw.set_map(glo_space=1,del_space=1)
	delivery_test_cw.delivery_path=delivery_test_cw.make_path('delivery',delivery_test_cw.delivery_map_num)

	delivery_test_cw.lane_width={'none':{3.0:[i for i in range(10)]}}
	delivery_test_cw.set_lanewidth()

	return delivery_test_cw


def delivery_test_ccw():
	space = 0.5
	# 0.5 (original) / 0.25 (x2) / 0.125 (x4) / 0.1 (x5)
	delivery_test_ccw=Path(path_map + "/src/delivery_test_ccw/global_"+str(space)+".pkl")
	delivery_test_ccw.set_link(list(np.array([0,190,215,330,370,540,590,700,744])*int(0.5/space)))
	delivery_test_ccw.set_dir([0,2,4,6,8,9],[1,3,5,7],[])
	
	delivery_test_ccw.delivery_map_num=2
	for i in range(delivery_test_ccw.delivery_map_num):
		del_route=path_map+"/src/delivery_test_ccw/delivery_"+str(i)+"_"+str(space)+".pkl"
		delivery_test_ccw.delivery_route.append(del_route)
		delivery_test_ccw.set_other_mode(mode='delivery', pc_route=del_route,link=i)	
	
	delivery_test_ccw.glo_to_del_start=list(np.array([60, 210])*int(0.5/space))
	delivery_test_ccw.glo_to_del_finish=list(np.array([80, 230])*int(0.5/space))
	delivery_test_ccw.glo_to_dynamic_start=400*int(0.5/space)
	delivery_test_ccw.glo_to_dynamic_finish=500*int(0.5/space)

	delivery_test_ccw.target_speed={'global':{'straight':10/3.6, 'curve':8/3.6},'parking':8/3.6,'delivery':4/3.6}
	delivery_test_ccw.set_map(space)
	delivery_test_ccw.delivery_path=delivery_test_ccw.make_path('delivery',delivery_test_ccw.delivery_map_num)

	delivery_test_ccw.lane_width={'none':{3.0:[i for i in range(10)]}}
	delivery_test_ccw.set_lanewidth()

	return delivery_test_ccw

use_map=delivery_test_cw()
start_index=0