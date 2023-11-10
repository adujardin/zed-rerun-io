CMAKE_MINIMUM_REQUIRED(VERSION 3.5)
PROJECT(rerun_ptclouds)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_BUILD_TYPE "Release")

# Set the path to the rerun_c build.
set(RERUN_CPP_URL "https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip" CACHE STRING "URL to the rerun_cpp zip.")

# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL ${RERUN_CPP_URL})
FetchContent_MakeAvailable(rerun_sdk)

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
FILE(GLOB_RECURSE HDR_FILES include/*.hpp)

ADD_EXECUTABLE(${PROJECT_NAME} ${HDR_FILES} ${SRC_FILES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} rerun_sdk)