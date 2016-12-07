cmake_minimum_required(VERSION 2.8.3)
project(ukf_example_srvs)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
 message_generation)

 add_service_files(
   FILES
   ComputeCommand.srv
 )

generate_messages()

catkin_package(
  CATKIN_DEPENDS
    message_runtime
)
