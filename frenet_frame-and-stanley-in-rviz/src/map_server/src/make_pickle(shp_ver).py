#!/usr/bin/python
#-*- coding: utf-8 -*-

# python 3.7 으로 실행할것

import pickle
import numpy as np
import pandas as pd
import geopandas as gpd
from shapely.geometry import LineString, Point, Polygon
from scipy.interpolate import interp1d

def read_shp(pc_route):
	df=gpd.read_file(pc_route)
	df.to_crs(epsg=5179)
	coor_dict={}
	for i in range(len(df)):
		ty=(df.type == 'Polygon')
		if ty[0]==1:
			data=df['geometry'][i].exterior.xy
		else:
			data=df['geometry'][i].xy
		# data[0].reverse()
		# data[1].reverse()
		coor_dict[i]={'x':data[0], 'y':data[1]}
	coor_dict
	# coor_dict[-1]=coor_dict[len(coor_dict)-1]
	# del(coor_dict[len(coor_dict)-2])
	wayp_dict={}
	wx=[]
	wy=[]
	for i in range(len(coor_dict)):
		wx.append(coor_dict[i]['x'][0])
		wy.append(coor_dict[i]['y'][0])
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
 
	dxds = np.gradient(fx(s), s, edge_order=1)
	dyds = np.gradient(fy(s), s, edge_order=1)
	wyaw = np.arctan2(dyds, dxds)

	return {
		"x": fx(s),
		"y": fy(s),
		"yaw": wyaw,
		"s": s
	}
waypoints={}
waypoints[0]=read_shp('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/boong_interpolated/parking_0.shp' )

with open('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/boong_interpolated/parking_0.pkl', 'wb') as handle:
  pickle.dump(waypoints,handle, protocol=0)
  
with open('/home/mds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/kcity/parking_0.pkl', "rb") as f:
	file = pickle.load(f)