cmake_minimum_required (VERSION 2.8.11)
project (gridastar)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 11)

include_directories(src)

add_library(gridastar STATIC
    src/AStar2.cpp
    src/AStar2.h
    )

find_package(benchmark)

if(benchmark_FOUND)

    message("Google Benchmark found")
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

install(TARGETS gridastar
  RUNTIME DESTINATION bin COMPONENT bin
  ARCHIVE DESTINATION lib COMPONENT lib )

install(FILES src/AStar2.h DESTINATION include/gridastar )
install(FILES gridastar-config.cmake README.md LICENSE DESTINATION share/gridastar )

set(CPACK_GENERATOR DEB)
set(CPACK_PACKAGE_NAME "gridastar")
set(CPACK_PACKAGE_VENDOR "Eurecat")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Eurecat")
set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_PATCH})

include(CPack)
