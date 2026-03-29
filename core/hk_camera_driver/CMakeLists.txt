cmake_minimum_required(VERSION 3.8)
project(hk_camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d core imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(MVS_ROOT "/opt/MVS")
find_path(MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  PATHS "${MVS_ROOT}/include"
  NO_DEFAULT_PATH
  REQUIRED
)

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(MVS_LIBRARY_DIR "${MVS_ROOT}/lib/64")
else()
  set(MVS_LIBRARY_DIR "${MVS_ROOT}/lib/32")
endif()

find_library(MVS_CAMERA_CONTROL_LIB
  NAMES MvCameraControl
  PATHS "${MVS_LIBRARY_DIR}"
  NO_DEFAULT_PATH
  REQUIRED
)

generate_parameter_library(hk_camera_driver_parameters
  src/hk_camera_driver.yaml
)

add_library(${PROJECT_NAME}_component SHARED
  src/parameter_validation.cpp
  src/hk_camera_driver.cpp
)

target_compile_features(${PROJECT_NAME}_component PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_component PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  "${MVS_INCLUDE_DIR}"
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${PROJECT_NAME}_component PRIVATE -O2)
endif()

ament_target_dependencies(${PROJECT_NAME}_component
  cv_bridge
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_ros
)

target_link_libraries(${PROJECT_NAME}_component
  hk_camera_driver_parameters
  ${MVS_CAMERA_CONTROL_LIB}
  ${OpenCV_LIBS}
)

set_target_properties(${PROJECT_NAME}_component PROPERTIES
  BUILD_RPATH "${MVS_LIBRARY_DIR}"
  INSTALL_RPATH "${MVS_LIBRARY_DIR}"
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "hk_camera_driver::HkCameraDriver"
  EXECUTABLE hk_camera_driver_node
)

if(TARGET hk_camera_driver_node AND (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  target_compile_options(hk_camera_driver_node PRIVATE -O2)
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
    hk_camera_driver_parameters
    ${PROJECT_NAME}_component
  EXPORT hk_camera_driver_targets
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_include_directories(include)
ament_export_dependencies(
  cv_bridge
  generate_parameter_library
  geometry_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_ros
)
ament_export_targets(hk_camera_driver_targets HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_parameter_validation
    test/test_parameter_validation.cpp
    src/parameter_validation.cpp
  )

  if(TARGET test_parameter_validation)
    target_compile_features(test_parameter_validation PUBLIC cxx_std_17)
    target_include_directories(test_parameter_validation PUBLIC
      "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include>"
    )
    target_link_libraries(test_parameter_validation
      hk_camera_driver_parameters
    )
    ament_target_dependencies(test_parameter_validation
      rclcpp
    )
  endif()
endif()

ament_package()
