#include_directories(${OpenCV_INCLUDE_DIRS})


# boblib bgs test
add_executable(boblib_bgs_test "src/demo_bgs.cpp")
target_include_directories(boblib_bgs_test PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/api/include>")
target_link_libraries(boblib_bgs_test PUBLIC "${OpenCV_LIBS}" boblib)
set_target_properties(boblib_bgs_test PROPERTIES FOLDER "apps")
install(TARGETS boblib_bgs_test RUNTIME DESTINATION "bin" COMPONENT "apps")

# boblib hikvision test
# add_executable(boblib_hik_test "src/demo_hik.cpp")
# target_include_directories(boblib_hik_test PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/api/include>")
# target_link_libraries(boblib_hik_test PUBLIC "${OpenCV_LIBS}" boblib avcodec avutil swscale hcnetsdk)
# set_target_properties(boblib_hik_test PROPERTIES FOLDER "apps")
# install(TARGETS boblib_hik_test RUNTIME DESTINATION "bin" COMPONENT "apps")

# # boblib_video
# add_executable(boblib_demo_video "src/demo_main.cpp")
# target_include_directories(boblib_demo_video PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/api/include>")
# target_link_libraries(boblib_demo_video PUBLIC "${OpenCV_LIBS}" boblib)
# set_target_properties(boblib_demo_video PROPERTIES FOLDER "apps")
# install(TARGETS boblib_demo_video RUNTIME DESTINATION "bin" COMPONENT "apps")

# if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
#     target_link_libraries(boblib_demo_video PRIVATE TBB::tbb)
# endif ()

# # boblib_qhy_camera_loop
# add_executable(boblib_qhy_camera_loop "src/qhy_camera_loop.cpp")
# target_include_directories(boblib_qhy_camera_loop PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/api/include>")
# target_link_libraries(boblib_qhy_camera_loop PUBLIC "${OpenCV_LIBS}" boblib)
# set_target_properties(boblib_qhy_camera_loop PROPERTIES FOLDER "apps")
# install(TARGETS boblib_qhy_camera_loop RUNTIME DESTINATION "bin" COMPONENT "apps")

# if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
#     target_link_libraries(boblib_qhy_camera_loop PRIVATE TBB::tbb)
# endif ()

# set(VIDEOTEST ${CMAKE_SOURCE_DIR}/Dahua-20220901-184734.mp4)
# if (EXISTS ${VIDEOTEST})
#     message(STATUS "Copying Video Dahua-20220901-184734.mp4")
#     configure_file(${VIDEOTEST} "${CMAKE_BINARY_DIR}/bin/Dahua-20220901-184734.mp4" COPYONLY)
# endif ()
