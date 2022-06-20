# 2022Capstone_AutoDriving
2022 Capstone Design - Auto Driving

* 실행 순서 작성

1. Odometry 실행 순서
- roslaunch ublox_gps ublox_device.launch
- roslaunch ntrip_ros ntrip.launch
- rosrun imu_sensor get_vel.py
- rosrun gps_to_vo gps_to_vo.py



4. cmd_serial 통신 실행
- rosrun stauto_control PC_to_ERP42.py
