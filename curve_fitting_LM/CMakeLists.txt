SET(PROJECT_NAME CurveFitting)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(CMAKE_BUILD_TYPE Release)  # Release, Debug, RelWithDebInfo
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

option(BUILD_APPS "Build APPs for CurveFitting" YES)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.1.0 REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${OPENCV_INCLUDE_DIR}
)

LIST(APPEND LINK_LIBS
    ${OpenCV_LIBS}
)

add_subdirectory(backend)
add_subdirectory(utils)

if (BUILD_APPS)
    add_subdirectory(app)
endif()
