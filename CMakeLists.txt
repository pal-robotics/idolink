cmake_minimum_required(VERSION 2.8.3)
project(idolink_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  range_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES idolink
  CATKIN_DEPENDS roscpp range_msgs 
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(idolink_node src/idolink_node.cpp)

foreach(dir launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(TARGETS idolink_node
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

target_link_libraries(idolink_node
  ${catkin_LIBRARIES}
)
