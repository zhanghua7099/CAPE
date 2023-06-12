cmake_minimum_required(VERSION 2.8)

project(cape)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Release)

################################
## BUILDING

file(GLOB_RECURSE cpp_file "src/*.cpp")
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include/
)

add_library(${PROJECT_NAME} ${cpp_file})

target_link_libraries(
    ${PROJECT_NAME} 
    ${OpenCV_LIBS}
)

add_executable(cape_offline run_cape_offline.cpp)
target_link_libraries(cape_offline ${PROJECT_NAME})

add_executable(pipe_seg pipe_seg.cpp)
target_link_libraries(pipe_seg ${PROJECT_NAME})

################################
## INSTALLATION

set_property(TARGET ${PROJECT_NAME} PROPERTY INTERFACE_LINK_DIRECTORIES ${OpenCV_LIBRARY_DIR})

set_target_properties(${PROJECT_NAME} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_INSTALL_LIBDIR}
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_LIBDIR}
)

# install library
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# install headers
set(PUBLIC_HEADERS
    "CAPE.h"
    "CylinderSeg.h"
    "Histogram.h"
    "Params.h"
    "PlaneSeg.h"
)

install(FILES ${PUBLIC_HEADERS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

# create a CMake configuration
install(EXPORT ${PROJECT_NAME}
    DESTINATION share/${PROJECT_NAME}/cmake
    FILE ${PROJECT_NAME}Config.cmake
)
