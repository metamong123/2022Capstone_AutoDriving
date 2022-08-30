#!/usr/bin/python
#-*- coding: utf-8 -*-

import pickle
import sys
import rospkg
import numpy as np
from scipy.interpolate import interp1d
from object_msgs.msg import Object

rospack = rospkg.RosPack()
path_map = rospack.get_path("map_server")
path_frenet=rospack.get_path("cv_agents")
sys.path.append(path_frenet+"/src/")
sys.path.append(path_map + "/src/")

def interpolate_waypoints(wx, wy, s_init, space=0.5):
	_s = s_init
	s = [s_init]
	for i in range(1, len(wx)):
		prev_x = wx[i - 1]
		prev_y = wy[i - 1]
		x = wx[i]
		y = wy[i]

		dx = x - prev_x
		dy = y - prev_y

		_s = np.hypot(dx, dy)
		s.append(s[-1] + _s)

	fx = interp1d(s, wx,bounds_error=False)
	fy = interp1d(s, wy,bounds_error=False)
	# s = np.linspace(s_init, s[-1], num=int(s[-1] / space) + 1, endpoint=True)
	s=np.arange(s_init, s[-1], step=space)
	# print(s)

	dxds = np.gradient(fx(s), s, edge_order=1,)
	dyds = np.gradient(fy(s), s, edge_order=1)
	wyaw = np.arctan2(dyds, dxds)
	# print(fx(s))
	return {
		"x": fx(s),
		"y": fy(s),
		"yaw": wyaw,
		"s": s
	}

class MakingPath:
	def __init__(self):
		self.x = []
		self.y = []
		self.yaw = []

class Path:
	def __init__(self,pc_route):
		self.global_route=pc_route #global pkl파일 경로
		self.waypoints={'global':{},'horizontal_parking':{},'diagonal_parking':{},'delivery':{}}
		with open(pc_route, 'rb') as f:
			file=pickle.load(f)
			self.waypoints['global']=file[0]
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
		self.link_len={'global':[],'horizontal_parking':[],'diagonal_parking':[],'delivery':[]}
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

	def set_other_mode(self, mode='parking', pc_route="/src/frontier/parking_route.pkl", link=None):
		if not link==None:
			self.waypoints[mode][link]={}
			if mode=='diagonal_parking':
				with open(pc_route, 'rb') as f:
					file=pickle.load(f)
					self.waypoints[mode][link]=file[0]
					self.waypoints[mode][link+1]={}
					for i in self.waypoints[mode][link].keys():
						self.waypoints[mode][link+1][i]=list(reversed(self.waypoints[mode][link][i]))
			
			elif mode == 'horizontal_parking':
				with open(pc_route, 'rb') as f:
					file=pickle.load(f)
					self.waypoints[mode][link+1]={}
					self.waypoints[mode][link+1]=file[0]
					for i in self.waypoints[mode][link+1].keys():
						self.waypoints[mode][link][i]=list(reversed(self.waypoints[mode][link+1][i]))
			
			else:
				for i in range(self.delivery_map_num):
					with open(pc_route, 'rb') as f:
						file=pickle.load(f)
						self.waypoints[mode][link]=file[0]

		else:
			self.waypoints[mode][0]={}
			if mode=='diagonal_parking':
				with open(pc_route, 'rb') as f:
					self.waypoints[mode][0]=pickle.load(f)
					self.waypoints[mode][1]={}
					for i in self.waypoints[mode][0].keys():
						self.waypoints[mode][1][i]=list(reversed(self.waypoints[mode][0][i]))
			
			elif mode == 'horizontal_parking':
				with open(pc_route, 'rb') as f:
					self.waypoints[mode][1]={}
					self.waypoints[mode][1]=pickle.load(f)
					for i in self.waypoints[mode][1].keys():
						self.waypoints[mode][0][i]=list(reversed(self.waypoints[mode][1][i]))
			
			else:
				for i in range(self.delivery_map_num):
					with open(pc_route, 'rb') as f:
						self.waypoints[mode][0]=pickle.load(f)
    
	def set_global_link(self, waypoint_list=[0],mode='global'):
		wayp_len=len(self.waypoints[mode]['x'])
		waypoint_list.append(wayp_len)
		# waypoint_list.append(wayp_len)
		waypoint_list.remove(0)
		self.link_len['global']=waypoint_list

	def interpolate_map(self, mode='global', space=0.5, link=None):
		if link==None:
			if mode=='global':
				self.waypoints[mode]=interpolate_waypoints(self.waypoints[mode]['x'], self.waypoints[mode]['y'], space=space)
			else:
				for i in range(len(self.waypoints[mode])):
					self.waypoints[mode][i]=interpolate_waypoints(self.waypoints[mode][i]['x'], self.waypoints[mode][i]['y'], space=space)
				self.set_other_link()
		else:
			if mode=='global':
				new_waypoints={'x':[],'y':[],'yaw':[],'s':[]}
				interpol_wayp={}
				if link == 0:
					interpol_wayp=interpolate_waypoints(self.waypoints[mode]['x'][:self.link_len[mode][link]], self.waypoints[mode]['y'][:self.link_len['global'][link]],0, space=space)
					for i in self.waypoints[mode].keys():
						new_waypoints[i]=np.append(new_waypoints[i],interpol_wayp[i])
						new_waypoints[i]=np.append(new_waypoints[i],self.waypoints[mode][i][self.link_len[mode][link]:])
				else:
					interpol_wayp=interpolate_waypoints(self.waypoints[mode]['x'][self.link_len[mode][link-1]:self.link_len[mode][link]], self.waypoints[mode]['y'][self.link_len[mode][link-1]:self.link_len[mode][link]],self.waypoints[mode]['s'][self.link_len[mode][link-1]], space=space)
					for i in self.waypoints[mode].keys():
						new_waypoints[i]=np.append(new_waypoints[i],self.waypoints[mode][i][:self.link_len[mode][link-1]])
						new_waypoints[i]=np.append(new_waypoints[i],interpol_wayp[i])
						new_waypoints[i]=np.append(new_waypoints[i],self.waypoints[mode][i][self.link_len[mode][link]:])
				link_diff=len(interpol_wayp['x'])-(self.link_len[mode][link]-self.link_len[mode][link-1])
				self.link_len[mode]=self.link_len[mode][:link]+[i+link_diff for i in self.link_len[mode][link:]]
				self.waypoints[mode]={}
				self.waypoints[mode]=new_waypoints
			else:
				self.waypoints[mode][link]=interpolate_waypoints(self.waypoints[mode][link]['x'], self.waypoints[mode][link]['y'], 0,space=space)
				self.set_other_link()
	
	def set_dir(self, straight, left, right):
		self.link_dir={'straight':straight,'left':left,'right':right}
	
	def set_waypoint_range(self, waypoint_start_list=None, waypoint_finish_list=None):
		w_f_l=waypoint_finish_list
	
		if waypoint_start_list == None:
			w_s_l=[sw-8 for sw in waypoint_finish_list] # for stopline
		else:
			w_s_l=waypoint_start_list
	
		return w_s_l, w_f_l

	def set_other_link(self):
		for i in self.waypoints.keys():
			if not i == 'global':
				link_i=-1
				self.link_len[i]=[]
				for j in range(len(self.waypoints[i].keys())):

					if i == 'horizontal_parking':
						# self.link_len[i]=[]
						if j % 2 ==0:
							link_i=-1
							link_i+=len(self.waypoints[i][j]["x"])
							self.link_len[i].append(link_i)
						else:
							link_i+=len(self.waypoints[i][j]["x"])
							self.link_len[i].append(link_i)
					elif i =='diagonal_parking':
						# self.link_len[i]=[]
						if j % 2 ==0:
							link_i=-1
							link_i+=len(self.waypoints[i][j]["x"])
							self.link_len[i].append(link_i)
						else:
							link_i+=len(self.waypoints[i][j]["x"])
							self.link_len[i].append(link_i)
					elif i=='delivery':
						# self.link_len[i]=[]
						link_i=-1
						link_i+=len(self.waypoints[i][j]["x"])
						self.link_len[i].append(link_i)

	def make_path(self, mode, map_num):
		path={}
		if mode == 'horizontal_parking':
			for link_ind in range(0,map_num*2,2):
				way=MakingPath()
				way.x=self.waypoints[mode][link_ind]['x']
				way.y=self.waypoints[mode][link_ind]['y']
				way.yaw=self.waypoints[mode][link_ind]['yaw']
				path[link_ind]=[way.x, way.y, way.yaw] #짝수 후진 홀수 전진
		elif mode == 'diagonal_parking':
			for link_ind in range(0,map_num*2,2):
				way=MakingPath()
				way.x=self.waypoints[mode][link_ind]['x']
				way.y=self.waypoints[mode][link_ind]['y']
				way.yaw=self.waypoints[mode][link_ind]['yaw']
				path[link_ind/2]=[way.x, way.y, way.yaw] #짝수 전진 (홀수 후진)
		else:
			for link_ind in range(map_num):
				way=MakingPath()
				way.x=self.waypoints[mode][link_ind]['x']
				way.y=self.waypoints[mode][link_ind]['y']
				way.yaw=self.waypoints[mode][link_ind]['yaw']
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

def frontier():
	frontier=Path(path_map + "/src/frontier/curve_test.pkl")
	frontier.set_global_link([0,100,200,300,400,500,600,700,800,900,1000,1100])
	frontier.set_dir([0,1],[2],[3,4,5,6,7,8,9,10,11,12,13])
	frontier.target_speed={'global':{'straight':20/3.6, 'curve':20/3.6},'parking':8/3.6,'delivery':10/3.6}
	frontier.set_other_link()
	frontier.lane_width={'none':{3.0:[i for i in range(14)]}}
	frontier.set_lanewidth()
	return frontier

def kcity():
	kcity=Path(path_map + "/src/kcity/route.pkl")
	kcity.set_global_link([0,110,190,280,390,530,600,670,720,780,880,940,970,1010,1190,1220,1260,1300,1360,1450,1580,1600,1640,1730,1780,1850,1910,2060,2140,2190,2260,2400,2510,2670,2760,2880,2960,3138])
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
	kcity.set_other_link()
	kcity.diagonal_parking_path=kcity.make_path('diagonal_parking',kcity.diagonal_parking_map_num)
	kcity.delivery_path=kcity.make_path('delivery',kcity.delivery_map_num)
	return kcity

def delivery_test():
	delivery_test=Path(path_map + "/src/delivery_test/global.pkl")
	delivery_test.set_global_link([0,380,430,660,740,1080,1180,1400,1487])
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
	delivery_test.set_other_link()
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
	delivery_test_cw.set_global_link(link_list)
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
	delivery_test_cw.set_other_link(glo_space=1,del_space=1)
	delivery_test_cw.delivery_path=delivery_test_cw.make_path('delivery',delivery_test_cw.delivery_map_num)

	delivery_test_cw.lane_width={'none':{3.0:[i for i in range(10)]}}
	delivery_test_cw.set_lanewidth()

	return delivery_test_cw


def delivery_test_ccw():
	space = 0.5
	# 0.5 (original) / 0.25 (x2) / 0.125 (x4) / 0.1 (x5)
	delivery_test_ccw=Path(path_map + "/src/delivery_test_ccw/global_"+str(space)+".pkl")
	delivery_test_ccw.set_global_link(list(np.array([0,190,215,330,370,540,590,700,744])*int(0.5/space)))
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
	delivery_test_ccw.set_other_link(space)
	delivery_test_ccw.delivery_path=delivery_test_ccw.make_path('delivery',delivery_test_ccw.delivery_map_num)

	delivery_test_ccw.lane_width={'none':{3.0:[i for i in range(10)]}}
	delivery_test_ccw.set_lanewidth()

	return delivery_test_ccw

def boong_inter():
	offset_state = ""
	# offset_state : "", "_offset", "_offset2", "_offset3"
	# old maps : "_old", "_old_offset", "_old_offset2"

	boong=Path(path_map + "/src/boong_interpolated/global"+offset_state+".pkl")
	
	if offset_state == "_old_offset2":
		boong.set_global_link([0,116,400,505,920,1020,1560,1620])
	else:
		boong.set_global_link([0,116,400,505,920,1020,1560,1620])

	boong.set_dir([0,2,4,5,6,8],[],[1,3,5,7])
	
	boong.diagonal_parking_map_num=2
	for i in range(boong.diagonal_parking_map_num):
		park_route=path_map+"/src/boong_interpolated/parking"+offset_state+"_"+str(i)+".pkl"
		boong.diagonal_parking_route.append(park_route)
		boong.set_other_mode(mode='diagonal_parking', pc_route=park_route,link=2*i)
	
	boong.glo_to_diagonal_park_start=10
	boong.glo_to_diagonal_park_finish=12
	# boong.parking_stop=[]
	# boong.park_to_glo_start=[]
	# boong.park_to_glo_finish=[]
	# boong.glo_to_del_start=[]
	# boong.glo_to_del_finish=[]
	
	boong.target_speed={'global':{'straight':15/3.6, 'curve':12/3.6},'parking':8/3.6,'delivery':10/3.6}
	boong.set_other_link()
 
	# boong.interpolate_map(mode='global', space=0.5,link=1)
	boong.diagonal_parking_path=boong.make_path('diagonal_parking',boong.diagonal_parking_map_num)
	# boong.delivery_path=boong.make_path('delivery',boong.delivery_map_num)

	boong.lane_width={'none':{3.0:[i for i in range(11)]}}
	boong.set_lanewidth()
	return boong

def boong():
	global_state = "30cm"
	# global_state : "10cm", "30cm", "1m"
	parking_state = "20cm"
	# parking_state : "10cm", "20cm", "30cm"
	offset_state = ""
	# offset_state : "", "_offset"

	boong=Path(path_map + "/src/boong_interpolated/global_"+global_state+".pkl")
	
	if global_state == "10cm":
		boong.set_global_link([0,116,400,505,920,1020,1560,1620])
	elif global_state =="30cm":
		boong.set_global_link([0,110,220,320,460,560,740,800])	
	elif global_state == "1m":
		boong.set_global_link([0,110,150,250,300,380,430,500])
	boong.set_dir([0,1,3,4,5,8,9],[],[2,4,6,7])
	
	boong.diagonal_parking_map_num=6
	for i in range(boong.diagonal_parking_map_num):
		park_route=path_map+"/src/boong_interpolated/parking"+offset_state+"_"+parking_state+"_"+str(i)+".pkl"
		boong.diagonal_parking_route.append(park_route)
		boong.set_other_mode(mode='diagonal_parking', pc_route=park_route,link=2*i)
	
	# boong.delivery_map_num=2
	# for i in range(boong.delivery_map_num):
	# 	del_route=path_map+"/src/boong/delivery_"+str(i)+".pkl"
	# 	boong.delivery_route.append(del_route)
	# 	boong.set_other_mode(mode='delivery', pc_route=del_route,link=i)	
	boong.glo_to_diagonal_park_start=10
	boong.glo_to_diagonal_park_finish=11
	
	if global_state == "10cm":
		boong.glo_to_dynamic_start=940			
		boong.glo_to_dynamic_finish=1020
	
	elif global_state == "30cm":
		boong.glo_to_dynamic_start=470
		boong.glo_to_dynamic_finish=550
		
	elif global_state == "1m":
		boong.glo_to_dynamic_start=300
		boong.glo_to_dynamic_finish=380
	# boong.parking_stop=[]
	# boong.park_to_glo_start=[]
	# boong.park_to_glo_finish=[]
	# boong.glo_to_del_start=[]
	# boong.glo_to_del_finish=[]
	
	boong.target_speed={'global':{'straight':15/3.6, 'curve':12/3.6},'parking':7/3.6,'delivery':10/3.6}
	boong.set_other_link()
	boong.diagonal_parking_path=boong.make_path('diagonal_parking',boong.diagonal_parking_map_num)
	# boong.delivery_path=boong.make_path('delivery',boong.delivery_map_num)
	for link_int in [1,3,5,7]:
		boong.interpolate_map(mode='global', space=1,link=link_int) # space는 m 단위로 넣기
	# boong.interpolate_map(mode='global', space=1,link=7)
	boong.lane_width={'none':{3.0:[i for i in range(11)]}}
	boong.set_lanewidth()
	return boong

use_map=boong()
start_index=0
if start_index==0:
    obj_msg=Object(x=use_map.waypoints['global']['x'][:use_map.link_len['global'][start_index]][0],y=use_map.waypoints['global']['y'][:use_map.link_len['global'][start_index]][0],yaw=use_map.waypoints['global']['yaw'][:use_map.link_len['global'][start_index]][0],v=0,L=1.600,W=1.04)
else:
	obj_msg=Object(x=use_map.waypoints['global']['x'][use_map.link_len['global'][start_index-1]:use_map.link_len['global'][start_index]][0],y=use_map.waypoints['global']['y'][use_map.link_len['global'][start_index-1]:use_map.link_len['global'][start_index]][0],yaw=use_map.waypoints['global']['yaw'][use_map.link_len['global'][start_index-1]:use_map.link_len['global'][start_index]][0],v=0,L=1.600,W=1.04)
obj_msg_gps=obj_msg
obj_msg_imu=obj_msg
