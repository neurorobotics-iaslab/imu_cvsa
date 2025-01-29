cmake_minimum_required(VERSION 3.0.2)
project(imu_cvsa)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")

list(INSERT CMAKE_MODULE_PATH
            0
            "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Xsens REQUIRED)

find_package(catkin REQUIRED
             COMPONENTS geometry_msgs
                        message_generation
                        roscpp
                        std_msgs
            )

add_message_files(
  FILES
    imu_data.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)
            
# Link directories
link_directories(${Xsens_LIBRARY_DIRS})

catkin_package( 
  INCLUDE_DIRS 
    include
    ${Xsens_INCLUDE_DIRS}
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    roscpp
    std_msgs
  LIBRARIES 
    ${Xsens_LIBRARIES}
    my_libraries
)

# Include directories
include_directories(
  include
  ${Xsens_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/imu_cvsa
)

# online CVSA library
add_library(lib_imu
                          src/Imu.cpp
                          )

target_link_libraries(lib_imu
									  ${catkin_LIBRARIES})

add_dependencies(lib_imu
				 ${catkin_EXPORTED_TARGETS}
)

###########
## Build ##
########### 
#add_executable(${PROJECT_NAME}_node
#               src/imu_old.cpp
#              )

add_executable(${PROJECT_NAME}_node
               src/imu_node.cpp
              )

target_link_libraries(${PROJECT_NAME}_node
                      ${catkin_LIBRARIES}
                      ${Xsens_LIBRARIES}
                      lib_imu
                    )

