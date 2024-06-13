cmake_minimum_required(VERSION 3.8)
project(curio_navigation)

find_package(ament_cmake REQUIRED) 
# find_package(roslaunch REQUIRED) 

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
