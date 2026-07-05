cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../../components/control/chassis"
    "${CMAKE_CURRENT_LIST_DIR}/../../components/control/web_control"
    "${CMAKE_CURRENT_LIST_DIR}/main"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(remote_box)
