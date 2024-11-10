cmake_minimum_required(VERSION 3.5)
project(spot_config)

find_package(ament_cmake REQUIRED)

message(${PROJECT_NAME})


install(
  DIRECTORY launch config maps worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()