***

WSL 2 에서 라이다 측정데이터를 이용하여 스캔영상을 그려주는 패
키지 lidarplot 를 완성하시오
/scan 토픽 구독 각도 거리 로 환산 스캔 영상 그리기 순서로
처리할 것
스캔 영상 그리기는 opencv 의 회전변환 또는 삼각함수를 이용
스캔 영상을 모니터에 출력하고 동시에 동영상 (mp 4 으로 저장할 것
Jetson nano 보드에서 sllidar_node 를 실행하고 lidarplot 패키지는
WSL 2 에서 작성하고 실행할것


***

설정 및 빌드

***

colcon build --packages-select lidarsim

source install/setup.bash

***

젝슨보드

***

창1

ros2 run lidarsim video_publisher

창2

ros2 run lidarsim sub

***

윈도우

***

창1

ros2 run lidarsim motor_cmd_publisher

창2

ros2 run lidarsim obstacle_avoidance
***

1. Rplidar C1의 좌표축을 설명하라.

각 거리값은 센서 중심(X=0, Y=0)에서 특정 각도(angle_min + angle_increment × i) 방향으로 측정된 거리입니다.

ROS의 LaserScan 메시지에서 각도는 반시계 방향으로 증가합니다.

***

2. Rplidar C1은 1초에 몇 번 토픽메시지를전송하는가?

ros2 topic hz /scan

average rate: 10.02

→ 1초에 약 10번(10Hz) /scan 메시지가 발행됨

***

3. 토픽메시지의크기(KB)는얼마인가?

ros2 topic bw /scan

Message size mean: 5.82 KB

→ 메시지 1개당 약 5.82KB

***

4. 메시지1개당몇개의거리측정값이포함되어있는가?

ros2 topic echo /scan

angle_min: -3.1415927410125732          # ← angle_min 값

angle_max: 3.1415927410125732           # ← angle_max 값

angle_increment: 0.008738784119486809   # ← angle_increment 값

(angle_max - angle_min) / angle_increment + 1


***

5. 1회전에몇번거리를측정하는가?
   
ros2 topic echo /scan

angle_min: -3.1415927410125732          # ← angle_min 값

angle_max: 3.1415927410125732           # ← angle_max 값

angle_increment: 0.008738784119486809   # ← angle_increment 값

(angle_max - angle_min) / angle_increment + 1

***

6. angle_min, angle_increment 값은 얼마인가?

ros2 topic echo /scan

angle_min: 일반적으로 -3.14159(라디안, -180도)

angle_increment: 일반적으로 0.00873(라디안, 약 0.5도)

***
