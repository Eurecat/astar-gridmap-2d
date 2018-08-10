cmake_minimum_required (VERSION 2.8.11)
project (gridastar)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

include_directories(src)

add_library(gridastar STATIC
    src/AStar2.cpp
    src/AStar2.h
    )

find_package(benchmark)

message("Google Benchmark found")

if(benchmark_FOUND)

    include_directories(${benchmark_INCLUDE_DIR})

    add_executable(benchmark-astar
        test/util.h
        test/benchmark.cpp
        )
    target_link_libraries(benchmark-astar
        gridastar
        ${benchmark_LIBRARIES}
        pthread)
else()
    message("Google Benchmark NOT found. It can be downloaded from https://github.com/google/benchmark")
endif()
