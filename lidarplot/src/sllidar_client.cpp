/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>

#define RAD2DEG(x) ((x)*180./M_PI)

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  int count = scan->scan_time / scan->time_increment;
  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
         RAD2DEG(scan->angle_max));

  for (int i = 0; i < count; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, scan->ranges[i]);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sllidar_client");

  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan", rclcpp::SensorDataQoS(), scanCb);

  rclcpp::spin(node);

  rclcpp::shutdown();


  return 0;
}
*/
#include "rclcpp/rclcpp.hpp"                         // ROS2 C++ 클라이언트 라이브러리
#include "sensor_msgs/msg/laser_scan.hpp"            // LaserScan 메시지 타입
#include "opencv2/opencv.hpp"                        // OpenCV 라이브러리
#include <cmath>                                     // 수학 함수

#define IMAGE_SIZE 600                               // 이미지 크기 (600x600)
#define SCALE 60                                     // 1m당 픽셀 수 (스케일)

class SllidarClient : public rclcpp::Node {
public:
  SllidarClient() : Node("sllidar_client") {
    // 비디오 라이터 초기화 (MP4, 30fps, 600x600)
    video_writer_.open("lidar_scan.mp4", 
                      cv::VideoWriter::fourcc('m','p','4','v'), 
                      30, 
                      cv::Size(IMAGE_SIZE, IMAGE_SIZE));
    // /scan 토픽 구독 (LaserScan 메시지)
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, 
      std::bind(&SllidarClient::scanCallback, this, std::placeholders::_1));
  }

  ~SllidarClient() {
    video_writer_.release();                         // 비디오 라이터 해제
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // 흰색 배경의 이미지 생성
    cv::Mat image = cv::Mat::zeros(IMAGE_SIZE, IMAGE_SIZE, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255));         // 흰색 배경

    // 센서 중심점 좌표
    cv::Point center(IMAGE_SIZE/2, IMAGE_SIZE/2);

    // 파란색 십자(+) 그리기
    int cross_len = 8;
    cv::line(image, cv::Point(center.x - cross_len, center.y), cv::Point(center.x + cross_len, center.y), cv::Scalar(255, 0, 0), 2);
    cv::line(image, cv::Point(center.x, center.y - cross_len), cv::Point(center.x, center.y + cross_len), cv::Scalar(255, 0, 0), 2);

    // 각 거리 데이터(각도, 거리) → 이미지 좌표 변환 및 점 찍기
    for(size_t i = 0; i < scan->ranges.size(); ++i) {
      float angle = scan->angle_min + scan->angle_increment * i; // 현재 각도(라디안)
      float distance = scan->ranges[i];                          // 현재 거리(m)

      // 유효한 거리만 사용 (무한대, 최대거리 이상 값은 무시)
      if(std::isinf(distance) || distance > scan->range_max) continue;

      // 극좌표 → 직교좌표 변환 (중심점 기준, y축은 아래로)
      int x = static_cast<int>(distance * SCALE * std::cos(angle) + center.x);
      int y = static_cast<int>(distance * SCALE * std::sin(angle) + center.y);

      // 이미지 범위 내에 있으면 빨간색 점으로 표시
      if(x >= 0 && x < IMAGE_SIZE && y >= 0 && y < IMAGE_SIZE) {
        cv::circle(image, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1); // 빨간 점
      }
    }

    // OpenCV 창에 이미지 출력
    cv::imshow("lidar", image);
    cv::waitKey(1); // 1ms 대기 (화면 갱신)

    // 프레임을 mp4 동영상으로 저장
    video_writer_ << image;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_; // 구독 객체
  cv::VideoWriter video_writer_;                                              // 비디오 저장 객체
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);                            // ROS2 초기화
  rclcpp::spin(std::make_shared<SllidarClient>());     // 노드 실행
  rclcpp::shutdown();                                  // ROS2 종료
  return 0;
}
