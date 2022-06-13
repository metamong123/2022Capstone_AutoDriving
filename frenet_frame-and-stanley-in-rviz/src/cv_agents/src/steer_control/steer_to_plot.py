import pandas as pd # python 3.6.9으로 실행할 것!
import matplotlib.pyplot as plt

## rostopic echo -b '/home/nsclmds/bagfiles/test_1/2022-06-13-21-05-57.bag' -p /ackermann_cmd > k_4_steer.csv

df=pd.read_csv('/home/nsclmds/catkin_ws/src/2022Capstone_AutoDriving/frenet_frame-and-stanley-in-rviz/src/cv_agents/src/steer_control/k_4_steer.csv')

df.columns
# df.loc[391]['field.drive.steering_angle']
x=df.loc[:]['%time']
y=df.loc[:]['field.drive.steering_angle']

plt.plot(x,y)
plt.show()