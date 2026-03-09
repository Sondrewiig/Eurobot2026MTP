cmake_minimum_required(VERSION 3.8)
project(sondre_bot_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds config materials
  DESTINATION share/${PROJECT_NAME}
)

ament_package()