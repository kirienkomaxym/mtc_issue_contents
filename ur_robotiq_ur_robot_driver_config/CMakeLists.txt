cmake_minimum_required(VERSION 3.5)
project(ur_robotiq_ur_robot_driver_config)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY hook
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY srdf
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
