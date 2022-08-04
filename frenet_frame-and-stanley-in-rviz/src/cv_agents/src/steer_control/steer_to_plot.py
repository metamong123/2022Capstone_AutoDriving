import pandas as pd # python 3.6.9으로 실행할 것! (터미널에 python3)
import matplotlib.pyplot as plt

## rostopic echo -b '/home/nsclmds/bagfiles/test_1/2022-06-13-21-05-57.bag' -p /ackermann_cmd > k_4_steer.csv

df=pd.read_csv('/home/mds/stanley/k05_sg5_wy08_wc08_yd07_thersh_015_0804_imu-all.csv')

df.columns
# df.loc[391]['field.drive.steering_angle']
x=df.iloc[:]['1659627507.25']
print(x)
y=df.loc[:]['-437.9024276643614']

plt.plot(x,y)
plt.show()