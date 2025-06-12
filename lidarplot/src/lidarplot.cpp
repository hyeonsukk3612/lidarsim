#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>

#define RAD2DEG(x) ((x)*180.0/M_PI)
#define IMAGE_SIZE 800  // 이미지 크기 (800x800)
#define SCALE 50        // 1m 당 픽셀 수

class LidarPlot : public rclcpp::Node {
public:
  LidarPlot() : Node("lidar_plot") {
    // OpenCV 비디오 라이터 초기화 (MP4 포맷)
    video_writer_.open("lidar_scan.mp4", 
                      cv::VideoWriter::fourcc('m','p','4','v'), 
                      30, 
                      cv::Size(IMAGE_SIZE, IMAGE_SIZE));

    // LaserScan 토픽 구독
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, 
      std::bind(&LidarPlot::scanCallback, this, std::placeholders::_1));
  }

  ~LidarPlot() {
    video_writer_.release();
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    // 이미지 생성 (흰 배경)
    cv::Mat image = cv::Mat::zeros(IMAGE_SIZE, IMAGE_SIZE, CV_8UC3);
    image.setTo(cv::Scalar(255, 255, 255));

    // 라이다 중심 좌표 계산
    cv::Point center(IMAGE_SIZE/2, IMAGE_SIZE/2);
    cv::circle(image, center, 5, cv::Scalar(0, 0, 255), -1);  // 중심점 표시

    // 각 거리 데이터 처리
    for(size_t i = 0; i < scan->ranges.size(); ++i) {
      float angle = scan->angle_min + scan->angle_increment * i;
      float distance = scan->ranges[i];

      // 유효한 거리 값만 처리
      if(std::isinf(distance) || distance > scan->range_max) continue;

      // 극좌표 → 직교좌표 변환
      int x = static_cast<int>(distance * SCALE * std::cos(angle) + center.x);
      int y = static_cast<int>(distance * SCALE * std::sin(angle) + center.y);

      // 좌표가 이미지 범위 내인지 확인
      if(x >= 0 && x < IMAGE_SIZE && y >= 0 && y < IMAGE_SIZE) {
        cv::circle(image, cv::Point(x, y), 2, cv::Scalar(0, 255, 0), -1);
      }
    }

    // 이미지 표시
    cv::imshow("Lidar Scan", image);
    cv::waitKey(1);

    // 동영상 저장
    video_writer_ << image;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  cv::VideoWriter video_writer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPlot>());
  rclcpp::shutdown();
  return 0;
}
