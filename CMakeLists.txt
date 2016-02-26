cmake_minimum_required(VERSION 2.8.3)
project(youbot_driver_ros_interface)

# enable python teleop script
catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS youbot_driver roscpp rosconsole tf pr2_msgs diagnostic_msgs geometry_msgs nav_msgs std_msgs std_srvs
message_generation)
find_package(Boost REQUIRED COMPONENTS thread date_time filesystem system)

add_service_files(
  FILES
  BaseSetPosition.srv
  BaseDisplace.srv
  BaseRotate.srv
)

generate_messages()

catkin_package(
	CATKIN_DEPENDS youbot_driver rosconsole roscpp tf pr2_msgs diagnostic_msgs geometry_msgs nav_msgs std_msgs std_srvs
)

## preprocessor definitions
ADD_DEFINITIONS(-DYOUBOT_CONFIGURATIONS_DIR=${youbot_driver_CONFIG_DIR})

## Set Compiler flags
INCLUDE_DIRECTORIES(
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
)	


add_executable(youbot_driver_ros_interface 
    src/youbot_oodl.cpp 
    src/YouBotOODLWrapper.cpp
    src/YouBotConfiguration.cpp
)

target_link_libraries(youbot_driver_ros_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(youbot_driver_ros_interface ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


## run sudo command to enable direct network access
## disabled here by default because it is set by Debian postinst command

OPTION(USE_SETCAP "Set permissions to access ethernet interface without sudo" OFF)

SET(SUDO_COMMAND sudo)
IF(NOT $ENV{USE_NORMAL_SUDO})
    SET(SUDO_COMMAND gksudo)
ENDIF(NOT $ENV{USE_NORMAL_SUDO})

IF(USE_SETCAP)
add_custom_command(TARGET youbot_driver_ros_interface POST_BUILD
    COMMAND ${SUDO_COMMAND} setcap cap_net_raw+ep $<TARGET_FILE:youbot_driver_ros_interface>
)    
endif(USE_SETCAP)


#test programs

install(TARGETS youbot_driver_ros_interface
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/examples/youbot_keyboard_teleop.py
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
