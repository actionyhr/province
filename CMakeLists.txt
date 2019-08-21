cmake_minimum_required(VERSION 3.0)

# Project Name
set(PROJ_NAME Test)
project(${PROJ_NAME})

# Check if c++11 is available
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Binary output path
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

FILE(GLOB HEADER_LIST "${CMAKE_SOURCE_DIR}/src/*.h")
FILE(GLOB SOURCE_LIST "${CMAKE_SOURCE_DIR}/src/*.cpp")

add_executable(${PROJ_NAME} ${SOURCE_LIST} ${HEADER_LIST})
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJ_NAME})

#set(OpenCV_DIR "C:/ST42Data/Code/opencv3Src/install/x86/vc15/lib") 
find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJ_NAME} ${OpenCV_LIBS})
endif()
# Realsense D435
set(EZECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

SET_PROPERTY(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJ_NAME})

find_package(realsense2 REQUIRED)

if(OpenCV_FOUND)
    include_directories(${realsense2_INCLUDE_DIRS})
    target_link_libraries(${PROJ_NAME} ${realsense2_LIBRARY})
    target_link_libraries(${PROJ_NAME} ${OpenCV_LIBS})
endif()
#pcl
find_package(PCL 1.8.1 REQUIRED)
 
if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    link_directories(${PCL_LIBRARY_DIRS})
    target_link_libraries(${PROJ_NAME} ${PCL_LIBRARIES})
endif()

