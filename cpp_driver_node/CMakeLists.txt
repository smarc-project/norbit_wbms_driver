cmake_minimum_required(VERSION 3.8)
project(norbit_wbms_cpp_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(norbit_wbms_interfaces REQUIRED)

add_executable(bathymetry_cpp_parser src/bathymetry_cpp_parser.cc)
ament_target_dependencies(bathymetry_cpp_parser rclcpp std_msgs norbit_wbms_interfaces)

install(TARGETS
    bathymetry_cpp_parser
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
