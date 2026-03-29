cmake_minimum_required(VERSION 3.8)
project(image_file_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc videoio)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

generate_parameter_library(image_file_publisher_parameters
    src/image_file_publisher.yaml
)

add_library(image_file_publisher_lib
    src/image_file_publisher_node.cpp
    src/parameter_validation.cpp
)

target_compile_features(image_file_publisher_lib PUBLIC cxx_std_17)
target_include_directories(image_file_publisher_lib PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(image_file_publisher_lib PRIVATE -O2)
endif()

ament_target_dependencies(image_file_publisher_lib
    cv_bridge
    rclcpp
    sensor_msgs
    std_msgs
)

target_link_libraries(image_file_publisher_lib
    image_file_publisher_parameters
    ${OpenCV_LIBS}
)

add_executable(image_file_publisher_node
    src/image_file_publisher_main.cpp
)

target_compile_features(image_file_publisher_node PUBLIC cxx_std_17)
target_link_libraries(image_file_publisher_node
    image_file_publisher_lib
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(image_file_publisher_node PRIVATE -O2)
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
        image_file_publisher_parameters
        image_file_publisher_lib
        image_file_publisher_node
    EXPORT image_file_publisher_targets
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
    std_msgs
)
ament_export_targets(image_file_publisher_targets HAS_LIBRARY_TARGET)

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
            image_file_publisher_parameters
        )
        ament_target_dependencies(test_parameter_validation
            rclcpp
        )
    endif()
endif()

ament_package()
