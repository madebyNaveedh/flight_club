cmake_minimum_required(VERSION 3.8)
project(nidar_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds
  models
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/teleop_keyboard.py
  scripts/teleop_keyboard
  scripts/generate_maze.py
  scripts/generate_maze
  scripts/autonomous_explorer.py
  scripts/autonomous_explorer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
