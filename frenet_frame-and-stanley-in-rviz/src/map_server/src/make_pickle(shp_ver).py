#!/usr/bin/python
#-*- coding: utf-8 -*-

# python 3.6 으로 실행할것

import pickle
import numpy as np
import pandas as pd
import geopandas as gpd
from shapely.geometry import LineString, Point, Polygon
from scipy.interpolate import interp1d

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

df=gpd.read_file('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/hightech_map/parking4/parking4.shp' )

# 좌표 변환이 필요한 경우 주석 삭제
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

wayp_dict={}
data_x=[]
data_y=[]
for i in range(len(df.geometry.x)):
  data_x.append(coor_dict[i]['x'][0])
  data_y.append(coor_dict[i]['y'][0])
# data_x=np.concatenate(data_x)
# data_y=np.concatenate(data_y)
waypoints=interpolate_waypoints(data_x,data_y,space=0.5)
wayp_dict[i]={'x':waypoints['x'],'y':waypoints['y'],'s':waypoints['s'],'yaw':waypoints['yaw']}


with open('parking4.pkl', 'wb') as handle:
  pickle.dump(wayp_dict,handle, protocol=0)
  
# with open('route.pkl', "rb") as f:
# 	waypoints = pickle.load(f)