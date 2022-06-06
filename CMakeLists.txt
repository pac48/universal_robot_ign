cmake_minimum_required(VERSION 3.5)
project(universal_robot_ign)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# ros package
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(IGNITION_VERSION fortress)
# ignition package
message(WARNING "$ENV{IGNITION_VERSION}")
# Citadel
if("$ENV{IGNITION_VERSION}" STREQUAL "citadel")
  find_package(ignition-transport8 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport8_VERSION_MAJOR})

  find_package(ignition-msgs5 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs5_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Citadel")
# Dome
elseif("$ENV{IGNITION_VERSION}" STREQUAL "dome")
  find_package(ignition-transport9 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport9_VERSION_MAJOR})

  find_package(ignition-msgs6 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs6_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Dome")
# Fortress
elseif("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-transport11 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

  find_package(ignition-msgs8 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Fortress")
# Default to Edifice
else()
  find_package(ignition-transport11 REQUIRED)
  set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

  find_package(ignition-msgs8 REQUIRED)
  set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Fortress")
endif()

find_package(ignition-gazebo6 REQUIRED)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})


# include
include_directories(include)

# generate universal_robot_ign lib
#AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)

set(DIR_SRCS src/ign_joint_subscriber.cpp src/ign_wrench_subscriber.cpp src/ign_joint_publisher.cpp src/ign_wrench_publisher.cpp)
add_library(universal_robot_ign
        SHARED
        ${DIR_SRCS})

#add_library(universal_robot_ign::universal_robot_ign ALIAS universal_robot_ign)

ament_target_dependencies(universal_robot_ign
    rclcpp 
    std_msgs
    sensor_msgs
    geometry_msgs
    trajectory_msgs
)

target_link_libraries(universal_robot_ign
  ignition-msgs${IGN_MSGS_VER}::core
  ignition-transport${IGN_TRANSPORT_VER}::core
)

#executable node
add_executable(joint_state_publisher src/joint_state_publisher_main.cpp src/joint_state_publisher.cpp)
target_link_libraries(joint_state_publisher
        universal_robot_ign)

#add_executable(joint_controller src/joint_controller_main.cpp)
#target_link_libraries(joint_controller universal_robot_ign)

# for ignition-gazebo plugin RobotiqController
add_library(RobotiqController SHARED plugins/robotiq_controller/RobotiqController.cpp)

target_link_libraries(RobotiqController
    ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
)

# for ignition-gazebo plugin WrenchControl
add_library(WrenchControl SHARED plugins/wrench_control/WrenchControl.cpp)

target_link_libraries(WrenchControl
        ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
        )

#install
install( DIRECTORY
    ign launch resource
  DESTINATION share/universal_robot_ign
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS universal_robot_ign
    EXPORT universal_robot_ign
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install executables
install(TARGETS joint_state_publisher  #joint_controller
  DESTINATION lib/universal_robot_ign
)

install(TARGETS RobotiqController 
    DESTINATION $ENV{HOME}/.ignition/gazebo/plugins
)

install(TARGETS WrenchControl
        DESTINATION $ENV{HOME}/.ignition/gazebo/plugins
        )

# Install executable scripts
install(PROGRAMS 
        scripts/moveit2.py
        scripts/test_pose_goal.py
        scripts/test_gripper.py
    DESTINATION lib/universal_robot_ign)

ament_export_include_directories(
        include
)

ament_export_dependencies(
        ignition-msgs8
        ignition-transport11
        rclcpp
)

#environment
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/ignition.dsv.in")

#test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
