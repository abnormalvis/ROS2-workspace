cmake_minimum_required(VERSION 3.5)
project(ignition_sim_pkg)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY world
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
