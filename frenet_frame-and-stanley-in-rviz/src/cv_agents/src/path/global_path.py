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
		self.nodes['parking']={}
		# self.nodes['delivery']={}
		with open(pc_route, 'rb') as f:
			self.nodes['global']=pickle.load(f)
		self.w={}
		self.parking_map_num=0
		self.delivery_map_num=0
		self.waypoints={}
		self.link_len={}
		self.link_dir={'straight':[],'left':[],'right':[]}
		self.target_speed={}
		self.stopline_start_list=[]
		self.stopline_finish_list=[]
		self.low_speed_start_list=[]
		self.low_speed_finish_list=[]
		self.glo_to_park_start=0
		self.glo_to_park_finish=0
		self.parking_stop=[]
		self.park_to_glo=[]
		self.lane_width={} # example lane_width={'3.3':[0],'3.8':[1],'4.1':[2], '6.6':[3]...}
	def set_other_mode(self, mode='parking', pc_route=path_map+"/src/frontier/parking_route.pkl", link=None):

		if not link==None:
			self.nodes[mode][link]={}
			if mode=='parking':
				with open(pc_route, 'rb') as f:
					file=pickle.load(f)
					self.nodes[mode][link]=file[0]
					self.nodes[mode][link+1]={}
				for i in self.nodes[mode][link].keys():
					self.nodes[mode][link+1][i]=list(reversed(self.nodes[mode][link][i]))
			
			else:
				with open(pc_route, 'rb') as f:
					file=pickle.load(f)
					self.nodes[mode][link]=file[0]
		
		else:
			self.nodes[mode][0]={}
			if mode=='parking':
				with open(pc_route, 'rb') as f:
					file=pickle.load(f)
					self.nodes[mode][0]=file[0]
					self.nodes[mode][link+1]={}
				for i in self.nodes[mode][0].keys():
					self.nodes[mode][1][i]=list(reversed(self.nodes[mode][0][i]))
		
			else:
				with open(pc_route, 'rb') as f:
					file=pickle.load(f)
					self.nodes[mode][0]=file[0]

	def set_link(self, waypoint_list=[0],mode='global'):
		
		nodes_len=len(self.nodes[mode])
		waypoint_list.append(nodes_len)
		
		for i in reversed(range(len(waypoint_list)-1)):
			self.nodes[mode][i]={'x':self.nodes[mode][0]['x'][waypoint_list[i]:waypoint_list[i+1]], 'y':self.nodes[mode][0]['y'][waypoint_list[i]:waypoint_list[i+1]], 's':self.nodes[mode][0]['s'][waypoint_list[i]:waypoint_list[i+1]], 'yaw':self.nodes[mode][0]['yaw'][waypoint_list[i]:waypoint_list[i+1]]}

	def set_dir(self, straight, left, right):
		self.link_dir={'straight':straight,'left':left,'right':right}
	
	def set_waypoint_range(self, waypoint_start_list=None, waypoint_finish_list=None):
		w_f_l=waypoint_finish_list
	
		if waypoint_start_list == None:
			w_s_l=[sw-10 for sw in waypoint_finish_list] # for stopline
		else:
			w_s_l=waypoint_start_list
	
		return w_s_l, w_f_l

	def set_map(self):
		for i in self.nodes.keys():
			self.w[i]={}
			self.waypoints[i]={}
			self.link_len[i]=[]

			if i == 'global':
				self.w[i]={'x':[],'y':[]}
				self.waypoints[i]={'x':[],'y':[],'yaw':[],'s':[]}
			
			for j in range(len(self.nodes[i].keys())):
				if i == 'parking':
					if j %2==0:
						self.w[i][j]={'x':[],'y':[]}
						self.waypoints[i][j]={'x':[],'y':[],'yaw':[],'s':[]}
				elif i == 'delivery':
					self.w[i][j]={'x':[],'y':[]}
					self.waypoints[i][j]={'x':[],'y':[],'yaw':[],'s':[]}

				for k in ('x','y'):
					if i == 'global':
						self.w[i][k].append(self.nodes[i][j][k][1:])
					elif i == 'parking':
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
					if i == 'parking':
						if j%2==0:
							self.w[i][j][k] = np.concatenate(self.w[i][j][k])
					elif i =='delivery':
						self.w[i][j][k] = np.concatenate(self.w[i][j][k])

			if i == 'global':
				self.waypoints[i] = interpolate_waypoints(self.w[i]['x'], self.w[i]['y'], space=0.5)
			elif i=='parking':
				for j in range(len(self.nodes[i].keys())):
					if j%2==0:
						self.waypoints[i][j] = interpolate_waypoints(self.w[i][j]['x'], self.w[i][j]['y'], space=0.5)
			elif i=='delivery':
				for j in range(len(self.nodes[i].keys())):
					self.waypoints[i][j] = interpolate_waypoints(self.w[i][j]['x'], self.w[i][j]['y'], space=0.5)

			link_i=-1
			for j in range(len(self.nodes[i].keys())):
				if i == 'parking':
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


def frontier():
	frontier=Path(path_map + "/src/frontier/route.pkl")
	frontier.set_link([0,100,200,300,400,500,600])
	frontier.target_speed=20/3.6
	frontier.set_map()
	return frontier

def kcity():
	kcity=Path(path_map + "/src/kcity/route.pkl")
	kcity.set_link([0,100,300,500,720,750,775,800,950,1000,1025,1060,1230,1280,1320,1360,1500,1600,1780,1820,1900,1960,2140,2190,2250,2300,2480,2550,2600,2750,3180])
	kcity.set_dir([0,1,2,3,4,7,8,9,11,12,14,16,20,22,23,24,25],[6,10,13,15,17],[5,18,19,21])    
	kcity.stopline_start_list,kcity.stopline_finish_list=kcity.set_waypoint_range(waypoint_finish_list=[260, 349, 459,737,947,1328,1522,1797,2153,2269,2485,2750,2846])
	kcity.parking_map_num=6
	park_ver="v1"
	for i in range(kcity.parking_map_num):
		park_route=path_map+"/src/kcity/parking_"+park_ver+"_"+str(i)+".pkl"
		kcity.set_other_mode(mode='parking', pc_route=park_route,link=2*i)
	kcity.glo_to_park_start=15
	kcity.glo_to_park_finish=18
	kcity.parking_stop=[]
	kcity.park_to_glo=[]
	# kcity.low_speed_start_list,kcity.low_speed_finish_list=kcity.set_waypoint_range(waypoint_finish_list=[260, 349, 459,737,947,1328,1522,1797,2153,2269,2485,2750,2846])
	kcity.target_speed={'high':20/3.6,'low':10/3.6}
	# kcity.lane_width={'3.3':[0],'3.8':[1],'4.1':[2], '6.6':[3]...}
	kcity.set_map()
	return kcity

if 16 > [15]:
	print("1")