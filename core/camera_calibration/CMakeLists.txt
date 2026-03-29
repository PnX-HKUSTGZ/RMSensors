cmake_minimum_required(VERSION 3.8)
project(camera_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d core highgui imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

generate_parameter_library(camera_calibration_parameters
    src/camera_calibration.yaml
)

add_library(camera_calibration_lib
    src/camera_calibration_node.cpp
)

target_compile_features(camera_calibration_lib PUBLIC cxx_std_17)
target_include_directories(camera_calibration_lib PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(camera_calibration_lib PRIVATE -O2)
endif()

ament_target_dependencies(camera_calibration_lib
    cv_bridge
    rclcpp
    sensor_msgs
)

target_link_libraries(camera_calibration_lib
    camera_calibration_parameters
    ${OpenCV_LIBS}
)

add_executable(camera_calibration_node
    src/camera_calibration_main.cpp
)

target_compile_features(camera_calibration_node PUBLIC cxx_std_17)
target_link_libraries(camera_calibration_node
    camera_calibration_lib
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(camera_calibration_node PRIVATE -O2)
endif()

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    TARGETS
        camera_calibration_parameters
        camera_calibration_lib
        camera_calibration_node
    EXPORT camera_calibration_targets
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

ament_export_include_directories(include)
ament_export_dependencies(
    cv_bridge
    generate_parameter_library
    rclcpp
    sensor_msgs
)
ament_export_targets(camera_calibration_targets HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
