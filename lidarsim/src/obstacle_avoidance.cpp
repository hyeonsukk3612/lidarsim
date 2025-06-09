#include <rclcpp/rclcpp.hpp>               // ROS2 C++ 노드 라이브러리
#include <sensor_msgs/msg/image.hpp>       // 이미지 메시지 타입
#include <std_msgs/msg/int16.hpp>          // int16 메시지 타입 (속도 제어용)
#include <cv_bridge/cv_bridge.h>           // ROS와 OpenCV 이미지 변환
#include <opencv2/opencv.hpp>               // OpenCV 라이브러리
#include <vector>
#include <queue>
using namespace std;
using namespace cv;

// BFS를 이용해 연결된 점들을 찾는 함수 (8방향 탐색)
void find_connected_points(const Mat &mask, Point start, vector<Point> &connected_points) {
    int rows = mask.rows;
    int cols = mask.cols;
    Mat visited = Mat::zeros(rows, cols, CV_8U); // 방문 여부 저장 행렬 초기화
    queue<Point> q;
    q.push(start);                              // 시작점 큐에 삽입
    visited.at<uchar>(start) = 1;               // 시작점 방문 처리
    connected_points.push_back(start);          // 결과 벡터에 시작점 추가

    // 8방향 좌표 이동 배열 (상하좌우 및 대각선)
    int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};

    while (!q.empty()) {
        Point p = q.front();
        q.pop();
        for (int i = 0; i < 8; ++i) {
            int nx = p.x + dx[i];
            int ny = p.y + dy[i];
            // 좌표가 이미지 범위 내에 있는지 확인
            if (nx >= 0 && nx < cols && ny >= 0 && ny < rows) {
                // 빨간색 영역(mask>0)이고 방문하지 않은 점이면
                if (mask.at<uchar>(ny, nx) > 0 && visited.at<uchar>(ny, nx) == 0) {
                    visited.at<uchar>(ny, nx) = 1; // 방문 처리
                    Point np(nx, ny);
                    connected_points.push_back(np); // 연결된 점 추가
                    q.push(np);                     // 큐에 삽입
                }
            }
        }
    }
}

// 특정 영역 내 빨간 점 처리 함수
void process_region(Mat &frame, const vector<Point> &region_contour, Point center, Point default_point, Scalar box_color, Point &closest_pt, double &dist_center) {
    Mat hsv, mask1, mask2, mask;
    cvtColor(frame, hsv, COLOR_BGR2HSV); // BGR -> HSV 변환
    // 빨간색 범위 1 (0~15도)
    inRange(hsv, Scalar(0, 20, 20), Scalar(15, 255, 255), mask1);
    // 빨간색 범위 2 (165~179도)
    inRange(hsv, Scalar(165, 20, 20), Scalar(179, 255, 255), mask2);
    mask = mask1 | mask2; // 두 범위 합침

    // 영역 마스크 생성 (영역 외부는 0, 내부는 255)
    Mat region_mask = Mat::zeros(mask.size(), CV_8U);
    vector<vector<Point>> contours = {region_contour};
    fillPoly(region_mask, contours, Scalar(255));

    // 영역 내 빨간 점만 추출
    Mat red_region;
    bitwise_and(mask, region_mask, red_region);

    vector<Point> points;
    findNonZero(red_region, points); // 빨간 점 좌표 추출

    if (points.empty()) {
        // 빨간 점 없으면 중심과 기본점 사이 파란 선 그리기
        line(frame, center, default_point, Scalar(255, 0, 0), 2);
        closest_pt = default_point;
        dist_center = norm(center - default_point); // 두 점 거리 계산
        return;
    }

    // 중심점(center)에서 가장 가까운 빨간 점 찾기
    Point closest;
    double min_dist = 1e9;
    for (const auto &pt : points) {
        double dist = norm(pt - center);
        if (dist < min_dist) {
            min_dist = dist;
            closest = pt;
        }
    }

    // 가장 가까운 점에서 연결된 점들 탐색 (BFS)
    vector<Point> connected_points;
    find_connected_points(red_region, closest, connected_points);

    // 연결된 점들의 무게중심 계산
    Point centroid(0, 0);
    for (const auto &pt : connected_points) {
        centroid += pt;
    }
    centroid.x /= (int)connected_points.size();
    centroid.y /= (int)connected_points.size();

    // 무게중심에서 가장 먼 점 찾기 (박스 크기 산정용)
    double max_dist = 0;
    Point farthest = centroid;
    for (const auto &pt : connected_points) {
        double dist = norm(pt - centroid);
        if (dist > max_dist) {
            max_dist = dist;
            farthest = pt;
        }
    }

    // 무게중심과 가장 먼 점 기준으로 박스 크기 계산
    int width = abs(farthest.x - centroid.x) * 2;
    int height = abs(farthest.y - centroid.y) * 2;
    Rect box(centroid.x - width/2, centroid.y - height/2, width, height);

    // 박스가 이미지 영역 밖으로 나가지 않도록 조정
    if (box.x < 0) box.x = 0;
    if (box.y < 0) box.y = 0;
    if (box.x + box.width > frame.cols) box.width = frame.cols - box.x;
    if (box.y + box.height > frame.rows) box.height = frame.rows - box.y;

    rectangle(frame, box, box_color, 2); // 초록색 박스 그리기

    line(frame, center, closest, Scalar(255, 0, 0), 2); // 중심과 가장 가까운 점 파란 선 연결

    closest_pt = closest;                  // 가장 가까운 점 반환
    dist_center = norm(center - closest); // 중심과 가장 가까운 점 거리 반환
}

class ObstacleAvoidance : public rclcpp::Node {
public:
    ObstacleAvoidance() : Node("obstacle_avoidance") {
        // 이미지 토픽 구독 (큐 크기 10)
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_topic", 10,
            std::bind(&ObstacleAvoidance::image_callback, this, std::placeholders::_1));
        // 속도 명령 퍼블리셔 생성
        vel_pub_ = this->create_publisher<std_msgs::msg::Int16>("/cmd_velocity", 10);
        writer_open_ = false; // 비디오 저장 초기화 플래그
        namedWindow("Obstacle Avoidance", WINDOW_NORMAL); // OpenCV 윈도우 생성
    }

    ~ObstacleAvoidance() {
        if (writer_open_) writer_.release(); // 비디오 파일 닫기
        destroyAllWindows();                   // 모든 OpenCV 윈도우 닫기
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // ROS 이미지 메시지를 OpenCV Mat으로 변환
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        Mat frame = cv_ptr->image;

        // (250,250)에서 (250,125)까지 검정색 선으로 영역 나누기
        line(frame, Point(250, 250), Point(250, 125), Scalar(0, 0, 0), 2);

        // 왼쪽 영역 좌표
        vector<Point> left_region = {Point(250,125), Point(125,125), Point(125,250), Point(250,250)};
        // 오른쪽 영역 좌표
        vector<Point> right_region = {Point(250,125), Point(375,125), Point(375,250), Point(250,250)};

        Point center(250, 250);          // 중심점
        Point left_default(125, 250);    // 왼쪽 기본점
        Point right_default(375, 250);   // 오른쪽 기본점

        Point left_closest, right_closest; // 각 영역 내 가장 가까운 빨간 점 좌표
        double left_dist, right_dist;       // 중심과 가장 가까운 점 거리

        // 왼쪽 영역 빨간 점 처리
        process_region(frame, left_region, center, left_default, Scalar(0, 255, 0), left_closest, left_dist);

        // 오른쪽 영역 빨간 점 처리
        process_region(frame, right_region, center, right_default, Scalar(0, 255, 0), right_closest, right_dist);

        // 더 먼 쪽으로 모터 제어 값 계산
        int base_speed = 100; // 기본 속도
        int Kp = 1;           // 비례 상수
        int velocity = base_speed;

        if (left_dist > right_dist) {
            // 왼쪽이 더 먼 쪽: 왼쪽으로 회피(오른쪽으로 회전)
            velocity = base_speed + int(Kp * (left_dist - right_dist));
        } else if (right_dist > left_dist) {
            // 오른쪽이 더 먼 쪽: 오른쪽으로 회피(왼쪽으로 회전)
            velocity = base_speed - int(Kp * (right_dist - left_dist));
        }
        // 거리가 같으면 직진 (velocity = base_speed)

        // 속도 명령 메시지 생성 및 발행
        std_msgs::msg::Int16 vel_msg;
        vel_msg.data = velocity;
        vel_pub_->publish(vel_msg);

        // 결과 영상 저장 (최초 1회만 열기)
        if (!writer_open_) {
            writer_.open("result_obstacle_avoidance.mp4", VideoWriter::fourcc('m','p','4','v'), 30,
                         Size(frame.cols, frame.rows));
            writer_open_ = true;
        }
        writer_.write(frame); // 프레임 저장

        // 화면 출력 (WSL2 환경에서는 X11 포워딩 필요)
        imshow("Obstacle Avoidance", frame);
        waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_; // 이미지 구독자
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr vel_pub_;            // 속도 명령 퍼블리셔
    VideoWriter writer_;                                                    // 비디오 저장 객체
    bool writer_open_;                                                      // 비디오 저장 활성화 여부
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);                              // ROS2 초기화
    rclcpp::spin(std::make_shared<ObstacleAvoidance>());  // 노드 실행 및 콜백 대기
    rclcpp::shutdown();                                    // ROS2 종료
    return 0;
}
