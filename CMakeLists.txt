cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StringMod.srv"
)

add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs tf2 tf2_ros geometry_msgs)

add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(server src/str_mod_service.cpp)
ament_target_dependencies(server rclcpp std_msgs)

install(TARGETS
  talker
  listener
  server
  DESTINATION lib/${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(talker "${cpp_typesupport_target}")
target_link_libraries(server "${cpp_typesupport_target}")

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)

  set(TESTFILES
    test/main.cpp
    test/test.cpp)

  ament_add_gtest(test_${PROJECT_NAME} ${TESTFILES})
  ament_target_dependencies(test_${PROJECT_NAME} rclcpp std_msgs)

  install(TARGETS
  test_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()