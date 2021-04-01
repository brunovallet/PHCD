# Persistance Homologique pour la Detection de Changements (PHDC) CMakeLists
# author: Bruno Vallet
# created: 16/07/2012

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(PHDC)
set(VERSION "0.0 beta")

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules")
find_package(OpenCV REQUIRED) #NO_CMAKE_PACKAGE_REGISTRY
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LINK_DIRECTORIES})

set(PHDC_SOURCES main.cpp ../Components.cpp ../Components.h ../CvUtils.cpp ../CvUtils.h)

add_executable(phdc ${PHDC_SOURCES})
target_link_libraries(phdc ${OpenCV_LIBRARIES})

if(CMAKE_VERBOSITY GREATER 0)
message(STATUS "Added target phdc (exe)")
endif(CMAKE_VERBOSITY GREATER 0)
