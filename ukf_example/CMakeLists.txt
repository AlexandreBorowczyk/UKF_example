cmake_minimum_required(VERSION 2.8.3)
project(ukf_example)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  ukf_example_srvs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


#message(${GAZEBO_INCLUDE_DIRS})
include_directories(include
  /usr/include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

generate_messages(
    DEPENDENCIES
    ukf_example_srvs
    )

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS
  roscpp

  DEPENDS
    roscpp
    gazebo_ros
    gazebo
    SDF
    message_runtime
    ukf_example_srvs
)

# Gazebo Plugin
add_library(pendulum_stabilization SHARED
    src/pendulum_stabilization_class.cpp
    )

add_dependencies(pendulum_stabilization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pendulum_stabilization
    ${GAZEBO_LIBRARIES}
    ${Boost_LIBRARIES}
    )

# SDRE controller
add_library(sdre_controller SHARED
    src/sdre_controller_class.cpp
    )
