#!/usr/bin/python
#-*- coding: utf-8 -*-

import pickle
import numpy as np
import pandas as pd
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
  # s = np.linspace(0, s[-1], num=int(s[-1] / space) + 1, endpoint=True)

  dxds = np.gradient(fx(s), s, edge_order=1)
  dyds = np.gradient(fy(s), s, edge_order=1)
  wyaw = np.arctan2(dyds, dxds)

  return {
    "x": fx(s),
    "y": fy(s),
    "yaw": wyaw,
    "s": s
  }

df=pd.read_csv('gps.csv')

data_x=[]
data_y=[]
for i in range(len(df.x)):
  data_x.append(df.loc[i].x)
  data_y.append(df.loc[i].y)
  
way_dict={}
waypoints=interpolate_waypoints(data_x,data_y,space=0.5)
way_dict[0]={'x':waypoints['x'],'y':waypoints['y'],'s':waypoints['s'],'yaw':waypoints['yaw']}

with open('/home/mds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/boong_old/parking_offset_0_0.5.pkl', 'wb') as handle:
  pickle.dump(way_dict,handle, protocol=0)
  
with open('/home/mds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/boong_old/parking_offset_0_0.5.pkl', "rb") as f:
	waypoints = pickle.load(f)