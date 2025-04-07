cmake_minimum_required(VERSION 3.5)

find_package(CUDA QUIET)
if (CUDA_FOUND)
#if (EXISTS "${OpenCV_INCLUDE_DIRS}/opencv2/core/cuda.hpp")
    SET(OPEN_CV_WITH_CUDA ON)
    message(STATUS "OpenCV was built with CUDA support!")
    project(boblib LANGUAGES CXX CUDA)
    set(CMAKE_CUDA_ARCHITECTURES 75)
    include_directories(${CUDA_INCLUDE_DIRS})
else()
    message(STATUS "OpenCV was not built with CUDA support")
    project(boblib LANGUAGES CXX)
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories($ENV{QHYCCD_SDK_INCLUDE_DIRS})

if(MSVC)
    target_compile_options(boblib PRIVATE /EHsc)
endif()

file(GLOB_RECURSE LIB_SOURCES_CPP "*.cpp")
file(GLOB_RECURSE LIB_SOURCES_CU "*.cu")

add_library(boblib STATIC ${LIB_SOURCES_CPP} ${LIB_SOURCES_CU})

target_link_libraries(boblib PRIVATE "${OpenCV_LIBS}" qhyccd)

if (CUDA_FOUND)
    target_compile_definitions(boblib PRIVATE HAVE_CUDA)
    set_target_properties(boblib PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_STANDARD 20
    )
endif()

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_libraries(boblib PRIVATE TBB::tbb)
endif ()

file(GLOB_RECURSE ALL_HEADER_FILES "*.h" "*.hpp")
foreach(_headerFile ${ALL_HEADER_FILES})
    get_filename_component(_dir ${_headerFile} PATH)
    file(RELATIVE_PATH _relative_dir ${CMAKE_SOURCE_DIR} ${_dir})
    list(APPEND RELATIVE_INCLUDE_DIRS ${_relative_dir})
    list(APPEND INCLUDE_DIRS ${_dir})
endforeach()
list(REMOVE_DUPLICATES RELATIVE_INCLUDE_DIRS)
list(REMOVE_DUPLICATES INCLUDE_DIRS)

foreach(_relative_dir ${RELATIVE_INCLUDE_DIRS})
    target_include_directories(boblib PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${_relative_dir}>
        $<INSTALL_INTERFACE:include/boblib/${_relative_dir}>
    )
endforeach()

install(TARGETS boblib
        EXPORT boblibTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)

foreach(_dir ${INCLUDE_DIRS})
    file(RELATIVE_PATH REL_DIR ${CMAKE_SOURCE_DIR} ${_dir})
    file(GLOB HEADER_FILES "${_dir}/*.h" "${_dir}/*.hpp")
    install(FILES ${HEADER_FILES} DESTINATION include/boblib/${REL_DIR})
endforeach()

install(EXPORT boblibTargets
        FILE boblibTargets.cmake
        DESTINATION lib/cmake/boblib)

# Include the CMakePackageConfigHelpers module
include(CMakePackageConfigHelpers)

# Generate the config file that is includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/boblibConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/boblibConfig.cmake"
    INSTALL_DESTINATION "lib/cmake/boblib")

# Generate the version file for the config file
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/boblibConfigVersion.cmake"
    VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}"
    COMPATIBILITY AnyNewerVersion)

# Install the configuration file
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/boblibConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/boblibConfigVersion.cmake
    DESTINATION lib/cmake/boblib)

# Setup the exports for the build tree
export(EXPORT boblibTargets FILE "${CMAKE_CURRENT_BINARY_DIR}/boblibTargets.cmake")
