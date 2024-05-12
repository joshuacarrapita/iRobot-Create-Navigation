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
	Walls.msg
    PDController.msg
)
#uncomment if you have defined services
#add_service_files(FILES
#)

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
    DEPENDENCIES std_msgs create_fundamentals red_fundamentals
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

add_executable(square_no_sensors src/square_no_sensors.cpp)
target_link_libraries(square_no_sensors ${catkin_LIBRARIES})
add_dependencies(square_no_sensors create_fundamentals_generate_messages_cpp)

add_executable(square_with_encoders src/square_with_encoders.cpp)
target_link_libraries(square_with_encoders ${catkin_LIBRARIES})
add_dependencies(square_with_encoders create_fundamentals_generate_messages_cpp)

add_executable(wanderer src/wanderer.cpp)
target_link_libraries(wanderer ${catkin_LIBRARIES})
add_dependencies(wanderer create_fundamentals_generate_messages_cpp)

add_executable(centering src/centering.cpp)
target_link_libraries(centering ${catkin_LIBRARIES})
add_dependencies(centering create_fundamentals_generate_messages_cpp)

add_executable(walls src/walls.cpp)
target_link_libraries(walls ${catkin_LIBRARIES})
add_dependencies(walls create_fundamentals_generate_messages_cpp)

add_executable(squ_w_enc_lightblue src/squ_w_enc_lightblue.cpp)
target_link_libraries(squ_w_enc_lightblue ${catkin_LIBRARIES})
add_dependencies(squ_w_enc_lightblue create_fundamentals_generate_messages_cpp)

add_executable(squ_w_enc_v2_lightblue src/squ_w_enc_v2_lightblue.cpp)
target_link_libraries(squ_w_enc_v2_lightblue ${catkin_LIBRARIES})
add_dependencies(squ_w_enc_v2_lightblue create_fundamentals_generate_messages_cpp)

add_executable(pd_test src/pd_test.cpp)
target_link_libraries(pd_test ${catkin_LIBRARIES})
add_dependencies(pd_test create_fundamentals_generate_messages_cpp)

add_library(moving src/moving.cpp)
target_link_libraries(square_with_encoders moving ${catkin_LIBRARIES})
add_dependencies(moving create_fundamentals_generate_messages_cpp)