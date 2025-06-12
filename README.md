***

장애물 회피알고리즘을 작성하고 lidarsave 패키지에서 저장한 동영상 파일을 이용하여 
시뮬레이션을 수행하는 lidarsim 패키지를 작성 스캔영상에서 에러 계산 영상처리 결과를 동영상으로 저장 (mp4)
에러를 이용하여 속도명령을 전송 다이내믹셀 구동
참고유튜브 https :://www youtube com/watch?v=HvWfm 4 Xtzbs
패키지의 소스코드와 실행결과 동영상 
(mp4 을 깃허브에 업로드 완료 후 강사에게 확인 받을 것)

![image](https://github.com/user-attachments/assets/5c7b5495-4093-40bd-92ce-33e0e2d06440)

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
