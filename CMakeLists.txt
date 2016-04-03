cmake_minimum_required(VERSION 2.8.3)
project(babs_slam)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next line to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)

#uncomment the following 4 lines to use the Eigen library
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library

add_library(babs_slam_lib 
src/babs_slam.cpp
src/update_map.cpp
src/sensor_callbacks.cpp)

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
set (SOURCEFILES /src/*.cpp PARENT_SCOPE)

cs_add_executable(sentry499_gazebo_state src/sentry499_gazebo_state.cpp)
cs_add_executable(sentry499_lidar src/sentry499_lidar.cpp)
cs_add_executable(babslam_main src/babslam_main.cpp)


#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_library(example my_lib)
target_link_libraries(babslam_main babs_slam_lib)

cs_install()
cs_export()
    
