cmake_minimum_required(VERSION 3.8)
project(gta_connecter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL 1.12 REQUIRED)

find_package(pcl_conversions REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(gta_msg REQUIRED)   

add_executable(gta_node src/pub_gta_pcl.cpp)
# 実行可能ファイルにライブラリをリンク
target_link_libraries(gta_node ${PCL_LIBRARIES})

add_executable(gta_map src/gta_map.cpp)
# 実行可能ファイルにライブラリをリンク
target_link_libraries(gta_map ${PCL_LIBRARIES})

add_executable(gta_save_pcd src/save_pcd.cpp)
# 実行可能ファイルにライブラリをリンク
target_link_libraries(gta_save_pcd ${PCL_LIBRARIES})

ament_target_dependencies(gta_node 
rclcpp 
sensor_msgs
pcl_conversions 
gta_msg
)

ament_target_dependencies(gta_map
  rclcpp
  sensor_msgs
  pcl_conversions
)

ament_target_dependencies(gta_save_pcd
  rclcpp
  sensor_msgs
  pcl_conversions
)

install(TARGETS
  gta_node
  gta_map
  gta_save_pcd
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  set(ament_cmake_copyright_FOUND TRUE)
  
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
