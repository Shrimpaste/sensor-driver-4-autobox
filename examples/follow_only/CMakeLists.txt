cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components/sensors/bu_uwb"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/sensors/imu_i2c"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/control/chassis"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(follow_only)
