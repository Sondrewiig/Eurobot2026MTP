cmake_minimum_required(VERSION 3.8)
project(main_bot_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds config materials
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
