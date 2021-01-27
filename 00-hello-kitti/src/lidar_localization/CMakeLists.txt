cmake_minimum_required(VERSION 2.8.3)
project(lidar_localization)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
  message_generation 
  std_srvs
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

set(ALL_TARGET_LIBRARIES "")
message( "this set(ALL_TARGET_LIBRARIES "") = ${ALL_TARGET_LIBRARIES}.")
include(cmake/geographic.cmake)
message( "this ALL_TARGET_LIBRARIES  include(cmake/geographic.cmake) = ${ALL_TARGET_LIBRARIES}.")
include(cmake/glog.cmake)
message( "this ALL_TARGET_LIBRARIES include(cmake/glog.cmake) = ${ALL_TARGET_LIBRARIES}.")
include(cmake/YAML.cmake)
include(cmake/PCL.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
include(cmake/global_defination.cmake)
catkin_package()
file(GLOB_RECURSE ALL_SRCS "*.cpp")
#message("GLOB_RECURSE ALL_SRCS  =${ALL_SRCS} ")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
#message("GLOB_RECURSE ALL_SRCS  =${NODE_SRCS} ")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
#message("GLOB_RECURSE ALL_SRCS  =${ALL_SRCS} ")
#将 00 中的所有的cpp文件编译成一个可执行文件,而为了确保只有一个node,进行了上面几行的操作.
add_executable(test_frame_node src/test_frame_node.cpp ${ALL_SRCS})
#将所有的库文件与可执行文件关联
#  ${catkin_LIBRARIES} 是通过 find_package()这个函数找到的所有的库. 在find_package之前次变量为空.
#  为什么要用两种办法,在我理解都是找到库
target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS 
        test_frame_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY 
        include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

## Mark other directories for installation:
install(DIRECTORY
        launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)
install(DIRECTORY
        config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)
install(DIRECTORY
        rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    FILES_MATCHING PATTERN "*.rviz"
)