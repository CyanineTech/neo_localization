cmake_minimum_required(VERSION 2.8.3)
project(neo_localization)

set(CMAKE_CXX_STANDARD 11)

## By default build with code optimization and no debug info
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release"
  	  CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            tf
            nav_msgs
			sensor_msgs
			geometry_msgs
            neo_common
            message_generation
            error_monitor_msgs
            error_monitor_srvs
            error_monitor
        )

add_message_files(
  FILES
  LocalizationStats.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/libcyanine/error_monitor/include
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        roscpp
        tf
        nav_msgs
        sensor_msgs
        geometry_msgs
        neo_common
        std_msgs
        message_runtime
        error_monitor_msgs
        error_monitor_srvs
        error_monitor
)

add_executable(neo_localization_node src/neo_localization_node.cpp)
# add_dependencies(neo_localization_node ${catkin_EXPORTED_TARGETS})
add_dependencies(neo_localization_node 
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp  
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(neo_localization_node  ${catkin_LIBRARIES} error_monitor_lib)

install(TARGETS neo_localization_node
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES LocalizationStats.msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)
