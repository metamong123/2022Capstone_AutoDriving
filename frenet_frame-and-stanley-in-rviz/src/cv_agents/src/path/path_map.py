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

def get_s_yaw(wx, wy):
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

	fx = interp1d(s, wx,bounds_error=False)
	fy = interp1d(s, wy,bounds_error=False)

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
			# for i in self.waypoints['global'].keys():
			# 	self.waypoints['global'][i]=list(reversed(self.waypoints['global'][i]))
			# self.waypoints['global']=get_s_yaw(self.waypoints['global']['x'],self.waypoints['global']['y'])
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
		self.trafficlight_list=[]
		self.notrafficlight_list=[]
		self.uturn_list=[]
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
		self.del_to_glo_start=[]
		self.del_to_glo_finish=[]
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
					self.waypoints[mode][link+1]=get_s_yaw(self.waypoints[mode][link+1]['x'],self.waypoints[mode][link+1]['y'])

			elif mode == 'horizontal_parking':
				with open(pc_route, 'rb') as f:
					file=pickle.load(f)
					self.waypoints[mode][link]=file[0]
					self.waypoints[mode][link+1]={}
					for i in self.waypoints[mode][link].keys():
						self.waypoints[mode][link+1][i]=list(reversed(self.waypoints[mode][link][i]))
					self.waypoints[mode][link+1]=get_s_yaw(self.waypoints[mode][link+1]['x'],self.waypoints[mode][link+1]['y'])
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
					self.waypoints[mode][1]=get_s_yaw(self.waypoints[mode][1]['x'],self.waypoints[mode][1]['y'])

			elif mode == 'horizontal_parking':
				with open(pc_route, 'rb') as f:
					self.waypoints[mode][0]=pickle.load(f)
					self.waypoints[mode][1]={}
					for i in self.waypoints[mode][0].keys():
						self.waypoints[mode][1][i]=list(reversed(self.waypoints[mode][0][i]))
					self.waypoints[mode][1]=get_s_yaw(self.waypoints[mode][1]['x'],self.waypoints[mode][1]['y'])
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
				link_rate=len(interpol_wayp['x'])/(self.link_len[mode][link]-self.link_len[mode][link-1])
				for i in range(len(self.low_speed_finish_list)):
					if self.low_speed_finish_list[i] >= self.link_len['global'][link]:
						self.low_speed_finish_list[i]+=link_diff
					elif self.low_speed_finish_list[i] >= self.link_len['global'][link-1]:
						self.low_speed_finish_list[i]=self.link_len['global'][link-1]+int((self.low_speed_finish_list[i]-self.link_len['global'][link-1])*link_rate)
				for i in range(len(self.low_speed_start_list)):
					if self.low_speed_start_list[i] >= self.link_len['global'][link]:
						self.low_speed_start_list[i]+=link_diff
					elif self.low_speed_start_list[i] >= self.link_len['global'][link-1]:
						self.low_speed_start_list[i]=self.link_len['global'][link-1]+int((self.low_speed_start_list[i]-self.link_len['global'][link-1])*link_rate)
				for i in range(len(self.uturn_list)):
					if self.uturn_list[i] >= self.link_len['global'][link]:
						self.uturn_list[i]+=link_diff
					elif self.uturn_list[i] >= self.link_len['global'][link-1]:
						self.uturn_list[i]=self.link_len['global'][link-1]+int((self.uturn_list[i]-self.link_len['global'][link-1])*link_rate)
				for i in range(len(self.notrafficlight_list)):
					if self.notrafficlight_list[i] >= self.link_len['global'][link]:
						self.notrafficlight_list[i]+=link_diff
					elif self.notrafficlight_list[i] >= self.link_len['global'][link-1]:
						# print("here")
						# print(link)
						# print(link_rate)
						self.notrafficlight_list[i]=self.link_len['global'][link-1]+int((self.notrafficlight_list[i]-self.link_len['global'][link-1])*link_rate)
				if self.glo_to_dynamic_start >= self.link_len['global'][link]:
					self.glo_to_dynamic_start+=link_diff
				elif self.glo_to_dynamic_start >= self.link_len['global'][link-1]:
					self.glo_to_dynamic_start=self.link_len['global'][link-1]+int((self.glo_to_dynamic_start-self.link_len['global'][link-1])*link_rate)
				if self.glo_to_dynamic_finish >= self.link_len['global'][link]:
					self.glo_to_dynamic_finish+=link_diff
				elif self.glo_to_dynamic_finish >= self.link_len['global'][link-1]:
					self.glo_to_dynamic_finish=self.link_len['global'][link-1]+int((self.glo_to_dynamic_finish-self.link_len['global'][link-1])*link_rate)
				if self.glo_to_horizontal_park_start >= self.link_len['global'][link]:
					self.glo_to_horizontal_park_start+=link_diff
				elif self.glo_to_horizontal_park_start >= self.link_len['global'][link-1]:
					self.glo_to_horizontal_park_start=self.link_len['global'][link-1]+int((self.glo_to_horizontal_park_start-self.link_len['global'][link-1])*link_rate)
				if self.glo_to_horizontal_park_finish >= self.link_len['global'][link]:
					self.glo_to_horizontal_park_finish+=link_diff
				elif self.glo_to_horizontal_park_finish >= self.link_len['global'][link-1]:
					self.glo_to_horizontal_park_finish=self.link_len['global'][link-1]+int((self.glo_to_horizontal_park_finish-self.link_len['global'][link-1])*link_rate)
				if self.glo_to_diagonal_park_finish >= self.link_len['global'][link]:
					self.glo_to_diagonal_park_finish+=link_diff
				elif self.glo_to_diagonal_park_finish >= self.link_len['global'][link-1]:
					self.glo_to_diagonal_park_finish=self.link_len['global'][link-1]+int((self.glo_to_diagonal_park_finish-self.link_len['global'][link-1])*link_rate)
				if self.glo_to_diagonal_park_start >= self.link_len['global'][link]:
					self.glo_to_diagonal_park_start+=link_diff
				elif self.glo_to_diagonal_park_start >= self.link_len['global'][link-1]:
					self.glo_to_diagonal_park_start=self.link_len['global'][link-1]+int((self.glo_to_diagonal_park_start-self.link_len['global'][link-1])*link_rate)
				for i in range(len(self.glo_to_del_start)):
					if self.glo_to_del_start[i] >= self.link_len['global'][link]:
						self.glo_to_del_start[i]+=link_diff
					elif self.glo_to_del_start[i] >= self.link_len['global'][link-1]:
						self.glo_to_del_start[i]=self.link_len['global'][link-1]+int((self.glo_to_del_start[i]-self.link_len['global'][link-1])*link_rate)
				for i in range(len(self.glo_to_del_finish)):
					if self.glo_to_del_finish[i] >= self.link_len['global'][link]:
						self.glo_to_del_finish[i]+=link_diff
					elif self.glo_to_del_finish[i] >= self.link_len['global'][link-1]:
						self.glo_to_del_finish[i]=self.link_len['global'][link-1]+int((self.glo_to_del_finish[i]-self.link_len['global'][link-1])*link_rate)
				for i in range(len(self.del_to_glo_finish)):
					if self.del_to_glo_finish[i] >= self.link_len['global'][link]:
						self.del_to_glo_finish[i]+=link_diff
					elif self.del_to_glo_finish[i] >= self.link_len['global'][link-1]:
						self.del_to_glo_finish[i]=self.link_len['global'][link-1]+int((self.del_to_glo_finish[i]-self.link_len['global'][link-1])*link_rate)
				for i in range(len(self.del_to_glo_start)):
					if self.del_to_glo_start[i] >= self.link_len['global'][link]:
						self.del_to_glo_start[i]+=link_diff
					elif self.del_to_glo_start[i] >= self.link_len['global'][link-1]:
						self.del_to_glo_start[i]=self.link_len['global'][link-1]+int((self.del_to_glo_start[i]-self.link_len['global'][link-1])*link_rate)
				for i in range(len(self.trafficlight_list)):
					if self.trafficlight_list[i] >= self.link_len['global'][link]:
						self.trafficlight_list[i]+=link_diff
					elif self.trafficlight_list[i] >= self.link_len['global'][link-1]:
						self.trafficlight_list[i]=self.link_len['global'][link-1]+int((self.trafficlight_list[i]-self.link_len['global'][link-1])*link_rate)
				self.link_len[mode]=self.link_len[mode][:link]+[i+link_diff for i in self.link_len[mode][link:]]
				self.waypoints[mode]={}
				self.waypoints[mode]=new_waypoints
				
			else:
				self.waypoints[mode][link]=interpolate_waypoints(self.waypoints[mode][link]['x'], self.waypoints[mode][link]['y'], 0,space=space)
				self.set_other_link()
	
	def set_dir(self, straight, left, right, uturn):
		self.link_dir={'straight':straight,'left':left,'right':right,'uturn':uturn}
	
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
			for link_ind in range(0,map_num*2):
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

def delivery_test_cw():
	del_space = 0.5
	glo_space=0.5
	# 0.5 (original) / 0.25 (x2) / 0.125 (x4) / 0.1 (x5)
	delivery_test_cw=Path(path_map + "/src/delivery_test_cw/global_"+str(glo_space)+".pkl")
	link_list=list(map(int,list(np.array([0,180,210,340,360,540,570,690,717])*float(0.5/1))))
	delivery_test_cw.set_global_link(link_list)
	delivery_test_cw.set_dir([0,2,4,6,8],[],[1,3,5,7],[])
	
	delivery_test_cw.delivery_map_num=2
	for i in range(delivery_test_cw.delivery_map_num):
		del_route=path_map+"/src/delivery_test_cw/delivery_"+str(i)+"_"+str(del_space)+".pkl"
		delivery_test_cw.delivery_route.append(del_route)
		delivery_test_cw.set_other_mode(mode='delivery', pc_route=del_route,link=i)	
	delivery_test_cw.glo_to_del_start=[30,110]
	delivery_test_cw.glo_to_del_finish=[40,120]
	delivery_test_cw.del_to_glo_start=[80,160]
	delivery_test_cw.del_to_glo_finish=[90,170]
	# delivery_test_cw.glo_to_del_start=list(np.array([30, 110])*float(0.5/1))
	# delivery_test_cw.glo_to_del_finish=list(np.array([40, 120])*float(0.5/1))
	delivery_test_cw.glo_to_dynamic_start=400
	delivery_test_cw.glo_to_dynamic_finish=500
	
	delivery_test_cw.target_speed={'global':{'straight':15/3.6, 'curve':12/3.6},'diagonal_parking':{'straight':7/3.6},'horizontal_parking':{'straight':7/3.6},'delivery':{'straight':5/3.6},'dynamic_object':{'straight':10/3.6}}
	delivery_test_cw.set_other_link()
	delivery_test_cw.delivery_path=delivery_test_cw.make_path('delivery',delivery_test_cw.delivery_map_num)

	delivery_test_cw.lane_width={'none':{3.0:[i for i in range(10)]}}
	delivery_test_cw.set_lanewidth()

	return delivery_test_cw

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
	boong.set_dir([0,1,3,4,5,8,9],[],[2,4,6,7],[])
	
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
	
	boong.target_speed={'global':{'straight':15/3.6, 'curve':12/3.6},'diagonal_parking':{'straight':7/3.6},'horizontal_parking':{'straight':7/3.6},'delivery':{'straight':10/3.6},'dynamic_object':{'straight':10/3.6}}
	boong.set_other_link()
	boong.diagonal_parking_path=boong.make_path('diagonal_parking',boong.diagonal_parking_map_num)
	# boong.delivery_path=boong.make_path('delivery',boong.delivery_map_num)
	# for link_int in [1,3,5,7]:
	# 	boong.interpolate_map(mode='global', space=1,link=link_int) # space는 m 단위로 넣기
	# # boong.interpolate_map(mode='global', space=1,link=7)
	boong.lane_width={'none':{3.0:[i for i in range(11)]}}
	boong.set_lanewidth()
	return boong

def kcity():
	kcity=Path(path_map + "/src/kcity/global.pkl")
	kcity.set_global_link([0,155,209,279,341,663,742,772,829,1214,1243,1678,1957,2120,2365,2738,2833,3000,3058,3133])
	kcity.set_dir([0,1,2,3,6,7,9,15,16,17,18,19,20],[8,10,11],[4,5,13,14],[12]) #마지막에 유턴링크 넣으세요

	kcity.horizontal_parking_map_num=2
	for i in range(kcity.horizontal_parking_map_num):
		park_route=path_map+"/src/kcity/parking_"+str(i)+".pkl"
		kcity.horizontal_parking_route.append(park_route)
		kcity.set_other_mode(mode='horizontal_parking', pc_route=park_route,link=2*i)
	
	kcity.delivery_map_num=2
	for i in range(kcity.delivery_map_num):
		del_route=path_map+"/src/kcity/delivery_"+str(i)+".pkl"
		kcity.delivery_route.append(del_route)
		kcity.set_other_mode(mode='delivery', pc_route=del_route,link=i)

	kcity.notrafficlight_list=[2339,2608]
	kcity.trafficlight_list=[205,275,738,1221,2829,2996,3054]
	kcity.uturn_list=[1991]

	kcity.glo_to_horizontal_park_start=3201
	kcity.glo_to_horizontal_park_finish=3211
	kcity.horizontal_parking_stop=[]
	kcity.horizontal_park_to_glo=[] 

	kcity.glo_to_del_start=[773, 2739]
	kcity.glo_to_del_finish=[775, 2741]
	kcity.del_to_glo_start=[809,2825]
	kcity.del_to_glo_finish=[811,2827]

	kcity.target_speed={'global':{'straight':15/3.6, 'curve':12/3.6, 'uturn':8/3.6},'diagonal_parking':{'straight':7/3.6},'horizontal_parking':{'straight':7/3.6},'delivery':{'straight':8/3.6},'dynamic_object':{'straight':10/3.6},'static_object':{'straight':7/3.6}}
	kcity.lane_width={'left':{3.3:{3.3:[7]}, 3.8:{3.3:[5]}}, 'right':{}, 'none':{3.3:[3,4,6,8,9,10,11,12,14,15,16], 3.8:[0,1,2,17,18,19,20], 4.1:[13]}}

	kcity.set_lanewidth()
	kcity.set_other_link()
	for link_int in [4,8,10,11,12,13,14]:
		kcity.interpolate_map(mode='global', space=0.2,link=link_int) # space는 m 단위로 넣기
	kcity.horizontal_parking_path=kcity.make_path('horizontal_parking',kcity.horizontal_parking_map_num)
	kcity.delivery_path=kcity.make_path('delivery',kcity.delivery_map_num)
	return kcity

def qualifier():
	qualifier=Path(path_map + "/src/qualifier/global.pkl")
	qualifier.set_global_link([0,96,124,154,219,457,480,654,754,901,1054])
	qualifier.set_dir([0,2,3,9,10],[4,5,6,7],[1,8],[])

	qualifier.diagonal_parking_map_num=6
	for i in range(qualifier.diagonal_parking_map_num):
		park_route=path_map+"/src/qualifier/parking_"+str(i)+".pkl"
		qualifier.diagonal_parking_route.append(park_route)
		qualifier.set_other_mode(mode='diagonal_parking', pc_route=park_route,link=2*i)

	qualifier.notrafficlight_list=[477,750]
	qualifier.trafficlight_list=[202]

	qualifier.glo_to_dynamic_start=662
	qualifier.glo_to_dynamic_finish=742

	qualifier.glo_to_diagonal_park_start=99
	qualifier.glo_to_diagonal_park_finish=102
	qualifier.diagonal_parking_stop=[]
	qualifier.diagonal_park_to_glo=[] 
	
	qualifier.target_speed={'global':{'straight':15/3.6, 'curve':12/3.6},'diagonal_parking':{'straight':7/3.6},'horizontal_parking':{'straight':7/3.6},'delivery':{'straight':10/3.6},'dynamic_object':{'straight':10/3.6},'static_object':{'straight':7/3.6}}
	qualifier.lane_width={'left':{3.3:{3.3:[5]}}, 'right':{}, 'none':{3.3:[4,6,7,8], 3.8:[0,1,2,3,9,10]}}

	qualifier.set_lanewidth()
	qualifier.set_other_link()
	qualifier.diagonal_parking_path=qualifier.make_path('diagonal_parking',qualifier.diagonal_parking_map_num)
	return qualifier

def uturn_test():
	uturn_test=Path(path_map + "/src/uturn_test/uturn_test.pkl")
	uturn_test.set_global_link([0,250,375,600])
	uturn_test.set_dir([0,3,4],[1,2],[],[])
	
	uturn_test.target_speed={'global':{'straight':8/3.6, 'curve':8/3.6, 'uturn':8/3.6},'diagonal_parking':{'straight':7/3.6},'horizontal_parking':{'straight':7/3.6},'delivery':{'straight':10/3.6},'dynamic_object':{'straight':10/3.6}}
	uturn_test.lane_width={'none':{3.3:[0,1,2,3,4]}}

	# uturn_test=Path(path_map + "/src/uturn_test/uturn_test2.pkl")
	# uturn_test.set_global_link([0,100,200,300,400])
	# uturn_test.set_dir([0,1,2,3,4,5],[],[])
	
	# uturn_test.target_speed={'global':{'straight':5/3.6, 'curve':5/3.6},'parking':5/3.6, 'delivery':5/3.6}
	# uturn_test.lane_width={'none':{3.3:[0,1,2,3,4,5]}}


	uturn_test.set_lanewidth()
	uturn_test.set_other_link()

	return uturn_test

def playground():

	playground=Path(path_map + "/src/playground/global.pkl")
	
	playground.set_global_link([0,48,152,198])
	playground.set_dir([0,2,4],[],[],[1,3])
	
	playground.glo_to_dynamic_start=160
	playground.glo_to_dynamic_finish=190

	# playground.delivery_map_num=2
	# for i in range(playground.delivery_map_num):
	# 	del_route=path_map+"/src/playground/delivery_"+str(i)+".pkl"
	# 	playground.delivery_route.append(del_route)
	# 	playground.set_other_mode(mode='delivery', pc_route=del_route,link=i)	

	# playground.glo_to_del_start=[12,169]
	# playground.glo_to_del_finish=[14,171]
	
	playground.target_speed={'global':{'straight':10/3.6, 'curve':10/3.6,'uturn':8/3.6},'diagonal_parking':{'straight':7/3.6},'horizontal_parking':{'straight':7/3.6},'delivery':{'straight':10/3.6},'dynamic_object':{'straight':10/3.6}}
	
	playground.set_other_link()
	playground.delivery_path=playground.make_path('delivery',playground.delivery_map_num)

	playground.lane_width={'none':{3.0:[i for i in range(5)]}}
	playground.set_lanewidth()
	return playground

def hightech_delivery():

	hightech_delivery=Path(path_map + "/src/hightech_delivery/global.pkl")
	
	hightech_delivery.set_global_link([0,10])
	hightech_delivery.set_dir([0,1,2],[],[],[])

	hightech_delivery.delivery_map_num=1
	for i in range(hightech_delivery.delivery_map_num):
		del_route=path_map+"/src/hightech_delivery/delivery_"+str(i)+".pkl"
		hightech_delivery.delivery_route.append(del_route)
		hightech_delivery.set_other_mode(mode='delivery', pc_route=del_route,link=i)

	hightech_delivery.glo_to_del_start=[14, 10000]
	hightech_delivery.glo_to_del_finish=[16, 10000]
	hightech_delivery.del_to_glo_start=[50, 10000]
	hightech_delivery.del_to_glo_finish=[52, 10000]


	hightech_delivery.target_speed={'global':{'straight':12/3.6, 'curve':10/3.6,'uturn':8/3.6},'diagonal_parking':{'straight':7/3.6},'horizontal_parking':{'straight':7/3.6},'delivery':{'straight':7/3.6},'dynamic_object':{'straight':10/3.6}}
	hightech_delivery.lane_width={'none':{3.0:[i for i in range(3)]}}
	hightech_delivery.set_lanewidth()
	hightech_delivery.set_other_link()

	hightech_delivery.delivery_path=hightech_delivery.make_path('delivery',hightech_delivery.delivery_map_num)


	return hightech_delivery

def hightech_parking():
	
	hightech_parking=Path(path_map + "/src/hightech_parking/global.pkl")
	
	hightech_parking.set_global_link([0,41,48,77,84])
	hightech_parking.set_dir([0,2,4,5],[],[1,3],[])

	hightech_parking.horizontal_parking_map_num=4
	for i in range(hightech_parking.horizontal_parking_map_num):
		park_route=path_map+"/src/hightech_parking/parking_"+str(i)+".pkl"
		hightech_parking.horizontal_parking_route.append(park_route)
		hightech_parking.set_other_mode(mode='horizontal_parking', pc_route=park_route,link=2*i)

	hightech_parking.glo_to_horizontal_park_start=[17,25,32,39]
	hightech_parking.glo_to_horizontal_park_finish=[19,27,34,41]
	hightech_parking.horizontal_parking_stop=[58]
	hightech_parking.horizontal_park_to_glo=[]
	
	hightech_parking.target_speed={'global':{'straight':12/3.6, 'curve':10/3.6,'uturn':8/3.6},'horizontal_parking':{'straight':5/3.6},'delivery':{'straight':10/3.6},'dynamic_object':{'straight':10/3.6}}
	hightech_parking.lane_width={'none':{2.0:[i for i in range(6)]}}
	hightech_parking.set_lanewidth()
	hightech_parking.set_other_link()
	hightech_parking.horizontal_parking_path=hightech_parking.make_path('horizontal_parking',hightech_parking.horizontal_parking_map_num)

	return hightech_parking

use_map=delivery_test_cw()
start_index=0

# use_map=kcity()
# start_index=12

if start_index==0:
    obj_msg=Object(x=use_map.waypoints['global']['x'][:use_map.link_len['global'][start_index]][0],y=use_map.waypoints['global']['y'][:use_map.link_len['global'][start_index]][0],yaw=use_map.waypoints['global']['yaw'][:use_map.link_len['global'][start_index]][0],v=0,L=1.600,W=1.04)
else:
	obj_msg=Object(x=use_map.waypoints['global']['x'][use_map.link_len['global'][start_index-1]:use_map.link_len['global'][start_index]][0],y=use_map.waypoints['global']['y'][use_map.link_len['global'][start_index-1]:use_map.link_len['global'][start_index]][0],yaw=use_map.waypoints['global']['yaw'][use_map.link_len['global'][start_index-1]:use_map.link_len['global'][start_index]][0],v=0,L=1.600,W=1.04)

obj_msg_gps=obj_msg
obj_msg_imu=obj_msg