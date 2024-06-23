# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(red_fundamentals)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp rospy sensor_msgs create_fundamentals message_generation)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
add_message_files(FILES
	WallsCombined.msg
	StopMsg.msg
	Cell.msg 
	Grid.msg
	Pose.msg
	Row.msg
	CalculateCenter.msg
)
#uncomment if you have defined services
add_service_files(FILES
	ExecutePlan.srv
	StopSrv.srv
	SimpleCall.srv
	Localization.srv
	MoveToPosition.srv
	Center.srv
	CalculateCenterService.srv
)

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)

#include_directories(${Boost_INCLUDE_DIRS})
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES std_msgs create_fundamentals
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
catkin_package(
    DEPENDS python-serial
	CATKIN_DEPENDS roscpp rospy sensor_msgs create_fundamentals message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(example_laser src/example_laser.cpp)
target_link_libraries(example_laser ${catkin_LIBRARIES})
add_dependencies(example_laser create_fundamentals_generate_messages_cpp)

add_executable(example_moving src/example_moving.cpp)
target_link_libraries(example_moving ${catkin_LIBRARIES})
add_dependencies(example_moving create_fundamentals_generate_messages_cpp)

#add_executable(square_no_sensors src/square_no_sensors.cpp)
#target_link_libraries(square_no_sensors ${catkin_LIBRARIES})
#add_dependencies(square_no_sensors create_fundamentals_generate_messages_cpp)

#add_executable(square_with_encoders src/square_with_encoders.cpp)
#target_link_libraries(square_with_encoders ${catkin_LIBRARIES})
#add_dependencies(square_with_encoders create_fundamentals_generate_messages_cpp)

add_executable(wanderer src/wanderer.cpp)
target_link_libraries(wanderer ${catkin_LIBRARIES})
add_dependencies(wanderer create_fundamentals_generate_messages_cpp)

#add_executable(centering src/centering.cpp)
#target_link_libraries(centering ${catkin_LIBRARIES})
#add_dependencies(centering create_fundamentals_generate_messages_cpp)

add_executable(walls src/walls.cpp)
target_link_libraries(walls ${catkin_LIBRARIES})
add_dependencies(walls create_fundamentals_generate_messages_cpp)

add_executable(stopper src/stopper.cpp)
target_link_libraries(stopper ${catkin_LIBRARIES})
add_dependencies(stopper create_fundamentals_generate_messages_cpp)

add_executable(execute_plan_server src/execute_plan_server.cpp)
add_library(moving src/moving.cpp)
add_library(paradriver src/paradriver.cpp)
add_library(song src/song.cpp)
add_library(center src/center.cpp)
add_library(smooth_plan src/smooth_plan.cpp)
target_link_libraries(execute_plan_server smooth_plan moving paradriver song center ${catkin_LIBRARIES})
add_dependencies(execute_plan_server create_fundamentals_generate_messages_cpp)

#add_executable(corridor src/corridor.cpp)
#target_link_libraries(corridor moving ${catkin_LIBRARIES})
#add_dependencies(corridor create_fundamentals_generate_messages_cpp)

#add_executable(pd_control src/pd_control.cpp)
#target_link_libraries(pd_control moving ${catkin_LIBRARIES})
#add_dependencies(pd_control create_fundamentals_generate_messages_cpp)

add_executable(smooth_server src/smooth_server.cpp)
target_link_libraries(smooth_server moving smooth_plan ${catkin_LIBRARIES})
add_dependencies(smooth_server create_fundamentals_generate_messages_cpp)

#add_executable(paradriver src/paradriver.cpp)
#target_link_libraries(paradriver ${catkin_LIBRARIES})
#add_dependencies(paradriver create_fundamentals_generate_messages_cpp)

add_executable(localization src/localization.cpp)
target_link_libraries(localization moving paradriver center song ${catkin_LIBRARIES})
add_dependencies(localization create_fundamentals_generate_messages_cpp)

add_executable(calculate_center src/calculate_center.cpp)             
target_link_libraries(calculate_center ${catkin_LIBRARIES})           
add_dependencies(calculate_center create_fundamentals_generate_messages_cpp) 

add_executable(walls_combined src/walls_combined.cpp)
target_link_libraries(walls_combined ${catkin_LIBRARIES})
add_dependencies(walls_combined create_fundamentals_generate_messages_cpp)

add_executable(test_sound src/test_sound.cpp)
target_link_libraries(test_sound ${catkin_LIBRARIES})
add_dependencies(test_sound create_fundamentals_generate_messages_cpp)

add_executable(center_service src/center_service.cpp)
target_link_libraries(center_service paradriver ${catkin_LIBRARIES})
add_dependencies(center_service create_fundamentals_generate_messages_cpp)

add_library(graph src/graph.cpp)
add_executable(move_to_position_server src/move_to_position_server.cpp)
target_link_libraries(move_to_position_server graph song ${catkin_LIBRARIES})
add_dependencies(move_to_position_server create_fundamentals_generate_messages_cpp)

add_executable(dry_run src/dry_run.cpp)
target_link_libraries(dry_run graph song ${catkin_LIBRARIES})
add_dependencies(dry_run create_fundamentals_generate_messages_cpp)

add_executable(calculate_center_service src/calculate_center_service.cpp)
target_link_libraries(calculate_center_service ${catkin_LIBRARIES})
add_dependencies(calculate_center_service create_fundamentals_generate_messages_cpp)
