cmake_minimum_required(VERSION 3.0.2)
project(suav_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES search
#  CATKIN_DEPENDS dji_osdk_ros roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

FILE(GLOB SRCS src/*.cpp)
foreach (src ${SRCS})
    message(${src})
    message("-------------------")
    get_filename_component(name ${src} NAME_WE)

    add_executable(${name} ${src})

    target_link_libraries(${name}
        ${catkin_LIBRARIES}
    )

endforeach()
