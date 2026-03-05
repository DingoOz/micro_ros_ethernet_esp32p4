cmake_minimum_required(VERSION 3.16)

# Use Ethernet transport, not WiFi
set(BUILD_WLAN_INTERFACE OFF CACHE BOOL "" FORCE)
set(BUILD_ETHERNET_INTERFACE ON CACHE BOOL "" FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(micro_ros_ethernet_esp32p4)
