cmake_minimum_required(VERSION 3.16)
project(recorded_stereo_mapping LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(nlohmann_json REQUIRED)
add_executable(orb-recording orb_bridge.cpp)
target_include_directories(orb-recording PRIVATE
  /opt/orb-slam3 /opt/orb-slam3/include /opt/orb-slam3/include/CameraModels
  /opt/orb-slam3/Thirdparty/Sophus ${Pangolin_INCLUDE_DIRS})
target_link_libraries(orb-recording PRIVATE
  /opt/orb-slam3/lib/libORB_SLAM3.so
  ${OpenCV_LIBS} Eigen3::Eigen ${Pangolin_LIBRARIES} nlohmann_json::nlohmann_json)
