cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
    message(FATAL_ERROR "In-source build not supported; generation should be done in a subdir, e.g.:  ${CMAKE_SOURCE_DIR}/build")
endif ()

list(APPEND CMAKE_MODULE_PATH
    "${CMAKE_SOURCE_DIR}/cmake/"
    "${CMAKE_SOURCE_DIR}/cmake/Modules/")

include(FrameworkUtils)

project(boblib CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_INSTALL_RPATH "$ORIGIN/../lib:$ORIGIN/")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Install path prefix (optional)")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Forced project build type" FORCE)
endif (NOT CMAKE_BUILD_TYPE)
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${CMAKE_CONFIGURATION_TYPES})
set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS "$<$<CONFIG:Debug>:DEBUG>")
set_property(DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS "$<$<CONFIG:Debug>:_DEBUG>")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

option(USE_LINK_TIME_OPTIM "Enable link time optimization" ON)
option(USE_FAST_MATH "Enable fast math optimization" OFF)
option(USE_OPENMP "Enable OpenMP in internal implementations" ON)
option(USE_CUDA "Enable CUDA support" OFF)

set(BOBLIB_APPS_COMPILE ON)

find_package(OpenCV 4.0 REQUIRED)
message(STATUS "Found OpenCV >=4.0")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (USE_OPENMP)
    find_package(OpenMP REQUIRED)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

if (USE_FAST_MATH)
    add_definitions(-ffast-math)
endif ()
if (WIN32)
    SET(CMAKE_CXX_FLAGS "-O2 -Ob2 -Oi -Ot -GL -Qpar -arch:AVX2 -fp:fast")
endif()
if (NOT WIN32)
    add_definitions(-O3)
    add_definitions(-march=native)

    add_definitions(-Wall)
    add_definitions(-Wno-deprecated)
    add_definitions(-Wextra)
    add_definitions(-Wfatal-errors)
    add_definitions(-Wshadow)
    add_definitions(-Werror)
    add_definitions(-pedantic-errors)
    add_definitions(-Wno-missing-braces)
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        add_definitions(-Ofast)
        find_package(TBB REQUIRED)  # for <execution> stdlib
    endif ()
endif()
if (USE_LINK_TIME_OPTIM)
    if (NOT WIN32)
        add_definitions(-flto)
    endif()
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        find_program(GCC_AR gcc-ar)
        if (GCC_AR)
            set(CMAKE_AR ${GCC_AR})
        endif ()
        find_program(GCC_RANLIB gcc-ranlib)
        if (GCC_RANLIB)
            set(CMAKE_RANLIB ${GCC_RANLIB})
        endif ()
    endif ()
endif ()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(CMAKE_SHARED_LINKER_FLAGS  "${CMAKE_SHARED_LINKER_FLAGS}  -Wl,--as-needed")
  set(CMAKE_EXE_LINKER_FLAGS     "${CMAKE_EXE_LINKER_FLAGS}     -Wl,--as-needed")
endif()

add_subdirectory(api)
if (BOBLIB_APPS_COMPILE)
    add_subdirectory(apps)
endif ()

