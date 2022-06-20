#!/usr/bin/python
#-*- coding: utf-8 -*-

import pickle
import numpy as np
import pandas as pd
from scipy.interpolate import interp1d
from pyproj import Proj, transform, CRS
 
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
df=pd.read_csv('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/map/hightech_parking1.txt',header=None)
# df=pd.read_csv('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/map/hightech_parking2.txt',header=None)
# df=pd.read_csv('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/map/hightech_parking3.txt',header=None)
# df=pd.read_csv('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/map/hightech_parking4.txt',header=None)

# df=pd.read_csv('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/map/parking1.txt',header=None)
# df=pd.read_csv('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/map/parking2.txt',header=None)
# df=pd.read_csv('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/map/parking3.txt',header=None)
# df=pd.read_csv('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/map/parking4.txt',header=None)
# df=pd.read_csv('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/map/parking5.txt',header=None)
# df=pd.read_csv('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/map/parking6.txt',header=None)
df.columns = ['위도','경도']

coord = np.array(df)

epsg5179= CRS("EPSG:5179")
wgs84=CRS('EPSG:4326')

	
converted = transform(wgs84, epsg5179, df['위도'].tolist(), df['경도'].tolist())

df['y']=converted[0]
df['x']=converted[1]

data_x=[]
data_y=[]
for i in range(len(df.x)):
  data_x.append(df.loc[i].x)
  data_y.append(df.loc[i].y)
  
way_dict={}
waypoints=interpolate_waypoints(data_x,data_y,space=0.5)
way_dict[0]={'x':waypoints['x'],'y':waypoints['y'],'s':waypoints['s'],'yaw':waypoints['yaw']}
# way_dict[0]['x']
with open('hightech_parking_map.pkl', 'wb') as handle:
  pickle.dump(way_dict,handle, protocol=0)
  
with open('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/parking_map/hightech_parking1.pkl', "rb") as f:
	park1_waypoints = pickle.load(f)
 
with open('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/parking_map/hightech_parking2.pkl', "rb") as f:
	park2_waypoints = pickle.load(f)

with open('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/parking_map/hightech_parking3.pkl', "rb") as f:
	park3_waypoints = pickle.load(f)

with open('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/map_server/src/parking_map/hightech_parking4.pkl', "rb") as f:
	park4_waypoints = pickle.load(f)


 
way_dict[0]={'x':park1_waypoints[0]['x'],'y':park1_waypoints[0]['y'],'s':park1_waypoints[0]['s'],'yaw':park1_waypoints[0]['yaw']}
way_dict[1]={'x':park2_waypoints[0]['x'],'y':park2_waypoints[0]['y'],'s':park2_waypoints[0]['s'],'yaw':park2_waypoints[0]['yaw']}
way_dict[2]={'x':park3_waypoints[0]['x'],'y':park3_waypoints[0]['y'],'s':park3_waypoints[0]['s'],'yaw':park3_waypoints[0]['yaw']}
way_dict[3]={'x':park4_waypoints[0]['x'],'y':park4_waypoints[0]['y'],'s':park4_waypoints[0]['s'],'yaw':park4_waypoints[0]['yaw']}