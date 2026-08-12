cmake_minimum_required(VERSION 3.8)
project(buddy_description)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(state_publisher src/state_publisher.cpp)
ament_target_dependencies(state_publisher rclcpp geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS state_publisher DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/buddy_serial_bridge.py      # UART link to the ESP32 motor controller
  scripts/buddy_person_detector.py    # OAK-D + yolov6-nano -> Detection3DArray
  scripts/buddy_face_recognizer.py    # InsightFace embeddings -> named faces
  scripts/buddy_face_db.py            # face embedding store (imported by above)
  scripts/buddy_enroll_face.py        # CLI: enroll a new person into face_db
  scripts/buddy_target_tracker.py     # picks + tracks the follow target
  scripts/buddy_follow_controller.py  # target pose -> cmd_vel
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
set(ament_cmake_copyright_FOUND TRUE)
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()
ament_package()
