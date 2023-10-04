
cmake_minimum_required(VERSION 3.8)
project(camera_publisher)
include(FetchContent)
FetchContent_Declare(
    libcamera-apps
    GIT_REPOSITORY https://github.com/raspberrypi/libcamera-apps.git
    GIT_TAG v1.2.2
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/externals/libcamera-apps
    )

if(NOT libcamera-apps_POPULATED)
    FetchContent_Populate(libcamera-apps)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(h264_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread program_options)
find_package (JPEG REQUIRED)
pkg_check_modules(libcamera REQUIRED libcamera)


configure_file(${libcamera-apps_SOURCE_DIR}/core/version.cpp.in version.cpp)
add_library(libcamera-apps OBJECT
    ${libcamera-apps_SOURCE_DIR}/encoder/encoder.cpp
    ${libcamera-apps_SOURCE_DIR}/encoder/h264_encoder.cpp
    ${libcamera-apps_SOURCE_DIR}/encoder/null_encoder.cpp
    ${libcamera-apps_SOURCE_DIR}/encoder/mjpeg_encoder.cpp
    ${libcamera-apps_SOURCE_DIR}/core/libcamera_app.cpp
    ${libcamera-apps_SOURCE_DIR}/core/options.cpp
    ${libcamera-apps_SOURCE_DIR}/core/post_processor.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/version.cpp
    ${libcamera-apps_SOURCE_DIR}/post_processing_stages/post_processing_stage.cpp
    ${libcamera-apps_SOURCE_DIR}/preview/preview.cpp
    ${libcamera-apps_SOURCE_DIR}/preview/null_preview.cpp
)

target_include_directories(libcamera-apps PUBLIC ${libcamera_INCLUDE_DIRS} ${libcamera-apps_SOURCE_DIR})
target_link_libraries(libcamera-apps ${libcamera_LINK_LIBRARIES} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${JPEG_LIBRARY})

add_executable(camera_publisher src/main.cpp)
target_link_libraries(camera_publisher libcamera-apps)
ament_target_dependencies(camera_publisher rclcpp std_msgs h264_msgs)
install(TARGETS
      camera_publisher
        DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
