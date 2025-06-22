cmake_minimum_required(VERSION 3.15)
project(Esp32Control VERSION 1.0 LANGUAGES CXX)
add_executable(esp32ctl main.cpp)
target_link_libraries(esp32ctl PRIVATE stdc++)