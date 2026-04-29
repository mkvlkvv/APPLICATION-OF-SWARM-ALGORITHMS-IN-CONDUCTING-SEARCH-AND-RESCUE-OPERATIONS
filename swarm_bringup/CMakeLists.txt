cmake_minimum_required(VERSION 3.8)
project(swarm_bringup)
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds scripts
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/sitl_multiple_run.sh
        DESTINATION lib/${PROJECT_NAME})

ament_package()
