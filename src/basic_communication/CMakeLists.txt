cmake_minimum_required(VERSION 3.8)
project(basic_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 패키지들에 대한 의존성을 찾습니다
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Publisher 실행 파일을 생성합니다
# 이 과정에서 컴파일러는 함수 포인터들을 실제 메모리 주소로 변환합니다
add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher
  rclcpp
  std_msgs)

# Subscriber 실행 파일을 생성합니다
# 여기서도 콜백 함수 포인터가 바이너리에 포함됩니다
add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber
  rclcpp
  std_msgs)

# 메모리 분석 노드를 생성합니다
# 이 노드는 rclcpp의 내부 메모리 구조를 분석하는 데 사용됩니다
add_executable(memory_analysis src/memory_analysis.cpp)
ament_target_dependencies(memory_analysis
  rclcpp
  std_msgs)

# 취약한 subscriber 노드를 생성합니다 (교육 목적)
# 경고: 이 노드는 의도적인 보안 취약점을 포함합니다
add_executable(vulnerable_subscriber src/vulnerable_subscriber.cpp)
ament_target_dependencies(vulnerable_subscriber
  rclcpp
  std_msgs)
# 취약한 코드에 대한 컴파일러 경고를 무시합니다 (교육 목적)
target_compile_options(vulnerable_subscriber PRIVATE -Wno-deprecated-declarations)

# 생성된 실행 파일들을 설치 디렉토리에 복사합니다
# 이렇게 하면 ros2 run 명령어로 실행할 수 있습니다
install(TARGETS
  publisher
  subscriber
  memory_analysis
  vulnerable_subscriber
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 다음 라인은 copyright 및 라이선스 체크를 건너뛰도록 설정합니다
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
