cmake_minimum_required(VERSION 2.8.3)
project(rplidar_ros)

add_compile_options(-std=c++11)

set(RPLIDAR_SDK_PATH "./sdk/")

find_package(catkin REQUIRED)
find_package(OpenCV REQUIRED)

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  cv_bridge
  rosconsole
  sensor_msgs
  image_transport
  std_msgs
)


add_message_files(FILES
  final_direction.msg
  inspection_value.msg
  )

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  INCLUDE_DIRS include 
  LIBRARIES opencv
  serial
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(rplidarNode src/node.cpp ${RPLIDAR_SDK_SRC})
target_link_libraries(rplidarNode ${catkin_LIBRARIES})

add_executable(rplidarNodeClient src/client.cpp)
target_link_libraries(rplidarNodeClient ${catkin_LIBRARIES})

install(TARGETS rplidarNode rplidarNodeClient
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)


add_executable(rp_point_calculation
  src/rp_point_calculation.cpp
)
add_dependencies(rp_point_calculation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rp_point_calculation
  ${catkin_LIBRARIES}
)


add_executable(inspection_handing
  src/inspection_handing.cpp
)
add_dependencies(inspection_handing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inspection_handing
  ${catkin_LIBRARIES}
)

add_executable(rp_degree_to_pointxy
  src/rp_degree_to_pointxy.cpp
)
add_dependencies(rp_degree_to_pointxy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rp_degree_to_pointxy
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
