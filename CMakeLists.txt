cmake_minimum_required(VERSION 2.8.3)
project(sandbox)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    std_srvs
    cmake_modules
)

find_package(TinyXML REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system program_options)
# find_package(PCL REQUIRED)
# find_package(OpenCV REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                     ROS MESSAGES AND SERVICES
# ------------------------------------------------------------------------------------------------

# Generate messages
# add_message_files(
#    FILES
#    message1.msg
#    ...
# )

# Generate services
# add_service_files(
#    FILES
#    service1.srv
#    ...
# )

# generate_messages(
#    DEPENDENCIES
#    geometry_msgs
#    ...
# )

# ------------------------------------------------------------------------------------------------
#                                          CATKIN EXPORT
# ------------------------------------------------------------------------------------------------

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bla
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS TinyXML
)

# ------------------------------------------------------------------------------------------------
#                                              BUILD
# ------------------------------------------------------------------------------------------------

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS}
)

# Base tester
add_executable(base_tester src/base_tester.cpp)
target_link_libraries(base_tester 
                      ${catkin_LIBRARIES}
                      ${TinyXML_LIBRARIES}
                      )
                      
                      # Base tester
add_executable(amcl_trigger src/amcl_trigger.cpp)
target_link_libraries(amcl_trigger 
                      ${catkin_LIBRARIES}
                      )
