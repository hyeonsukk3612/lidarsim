cmake_minimum_required(VERSION 3.5)
project(lidarplot)

# C 표준 및 C++ 표준 설정
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 컴파일 옵션 (경고 표시)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# sllidar_client.cpp 빌드 타겟 추가
add_executable(sllidar_client src/sllidar_client.cpp)  # 소스 파일 경로 주의!
ament_target_dependencies(sllidar_client rclcpp sensor_msgs OpenCV)

# 타겟 설치 설정
install(TARGETS sllidar_client
  DESTINATION lib/${PROJECT_NAME}
)

# 테스트 관련 (선택사항)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
