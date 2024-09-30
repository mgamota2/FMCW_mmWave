cmake_minimum_required(VERSION 3.8)
project(cpp_mmwavec)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(mmwave src/radar_system.cpp)
ament_target_dependencies(mmwave rclcpp std_msgs)

install(TARGETS
  mmwave
  DESTINATION lib/${PROJECT_NAME})


ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
