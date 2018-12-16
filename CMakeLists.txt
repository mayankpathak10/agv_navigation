cmake_minimum_required(VERSION 2.8.3)
project(agv_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gmapping
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  tf
  roslint
  nav_core
  costmap_2d
  base_local_planner
  pluginlib
  visualization_msgs
  costmap_2d
)


###################################
## catkin specific configuration ##
###################################


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES agv_navigation
  CATKIN_DEPENDS 
        gmapping 
        roscpp 
        rospy 
        geometry_msgs 
        std_msgs 
        sensor_msgs 
        message_generation
        pluginlib
        costmap_2d
        base_local_planner
        nav_core
        visualization_msgs
#  DEPENDS system_lib   
)

add_library(${PROJECT_NAME} 
  src/turtlebot_rrt.cpp
  src/vertex.cpp
)

#
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
# Including SDL into CMAKE
#INCLUDE(FindPkgConfig)
#PKG_SEARCH_MODULE(SDL2 REQUIRED sdl2)
#INCLUDE_DIRECTORIES(${SDL2_INCLUDE_DIRS})

#add_executable(${PROJECT_NAME}_node src/astar_navigator.cpp 
#									src/map.cpp 
#									src/nodes.cpp 
#									src/main.cpp
#                                    src/Explorer.cpp
#                                    src/Agv.cpp)
add_executable(${PROJECT_NAME}_node src/main.cpp
                                    src/Explorer.cpp
                                    src/Agv.cpp
                                    #src/agv_rrt.cpp
                                    #src/vertex.cpp
                                    )

                                #add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(${PROJECT_NAME}_node src/explorer.cpp)
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_agv_navigation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



if(CATKIN_ENABLE_TESTING)
   find_package(rostest REQUIRED)
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage") # enabling coverage
 add_rostest_gtest(AgvNavTest
 		    test/agvnavigatorTest.launch
                    test/AgvNavTest.cpp 
 		    test/ExplorerTest.cpp 
                    test/AgvTest.cpp
 		    test/vertex_test.cpp
		    test/rrtpath_test.cpp
                    src/Agv.cpp  
 		    src/Explorer.cpp
		    src/vertex.cpp
		    src/turtlebot_rrt.cpp
                   # src/agv_rrt.cpp
                   # src/vertex.cpp

                    )
  # add_rostest_gtest(rrt_test 
  target_link_libraries(AgvNavTest 
                    ${catkin_LIBRARIES})
  add_dependencies(
 	  AgvNavTest
 	  ${catkin_EXPORTED_TARGETS}
 )
 endif()



  install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES rrt_planner_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})  

