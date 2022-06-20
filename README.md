# 2022Capstone_AutoDriving
2022 Capstone Design - Auto Driving

* pickle file 생성
1. frenet_frame-and-stanley-in-rviz/src/map_server/src
2. txt, csv, shp 파일에 맞춰 make_pickle python 파일 열기 
3. 현재 차량에 얹는 PC에서는 python 3.6으로 열어야 import pandas 됨
(개인 컴퓨터로 할 경우 pandas만 있으면 상관 X)
4. df=pd.read_csv('') <-'' 안에 원하는 파일명 넣기
5. csv ver의 경우 좌표 변환 코드 없으므로 필요에 따라 다른 코드에서 복붙할 것 / txt 파일의 경우 위도, 경도 -> epsg:5179로의 좌표 변환 코드가 있으므로 필요에 따라 변경할 것
6. txt ver의 경우 way_dict key가 0,1,2,3으로 총 4개가 있는데 여러 경로를 각 다른 링크로 합칠 때 사용할 수 있도록 되어있음 필요에 따라 가감할 것
7. dump 부분이 pickle 파일 생성하는 부분
8. load 부분이 pickle 파일이 맞게 써졌는지 확인하


* 실행 순서 작성

1. Odometry 실행 순서
- roslaunch ublox_gps ublox_device.launch
- roslaunch ntrip_ros ntrip.launch
- rosrun imu_sensor get_vel.py
- rosrun gps_to_vo gps_to_vo.py



4. cmd_serial 통신 실행
- rosrun stauto_control PC_to_ERP42.py
