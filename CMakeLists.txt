# project
cmake_minimum_required (VERSION 3.0)
project (libviso2)

# directories
set (LIBVISO2_SRC_DIR src)

# use sse3 instruction set
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -fPIC -march=native")
SET(CMAKE_CXX_FLAGS "-msse3 -Wall -O3 -fPIC -march=native")
# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    add_definitions(-DCOMPILEDWITHC17)
    message(STATUS "Using flag -std=c++17.")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(PythonInterp 3 REQUIRED)
find_package(PythonLibs REQUIRED)
if (Python3_FOUND)
    if (Python3_FOUND)
        MESSAGE("Python3 Version: " ${Python3_VERSION})

    endif ()

    MESSAGE("Python3 Version: " ${Python3_VERSION})

endif ()


find_package(pybind11 REQUIRED)
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

include_directories( ${OpenCV_INCLUDE_DIRS} )

# include directory
include_directories(
        ${LIBVISO2_SRC_DIR}
        ${PYTHON_INCLUDE_DIRS}
        ${pybind11_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)


# sources
FILE(GLOB LIBVISO2_SRC_FILES "src/*.cpp")

# make release version
set(CMAKE_BUILD_TYPE Release)

# demo program
add_library(viso2 SHARED src/viso.cpp)

add_executable(kitti2015 ${LIBVISO2_SRC_FILES} examples/kitti2015.cpp)
target_link_libraries(kitti2015 png)

add_executable(kitti2012 ${LIBVISO2_SRC_FILES} examples/kitti2012.cpp)
target_link_libraries(kitti2012 png)

add_executable(drivingstereo ${LIBVISO2_SRC_FILES} examples/drivingstereo.cpp)
target_link_libraries(drivingstereo png ${OpenCV_LIBS})

add_executable(kittivo ${LIBVISO2_SRC_FILES} examples/kittivo.cpp)
target_link_libraries(kittivo png ${OpenCV_LIBS})
