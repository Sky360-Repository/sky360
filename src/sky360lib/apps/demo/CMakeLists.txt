include_directories(${OpenCV_INCLUDE_DIRS})

# sky360lib_video
add_executable(
    sky360lib_demo_video
        "src/demo_main.cpp"
)

target_include_directories(
    sky360lib_demo_video
        PUBLIC
            "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/api/include>"
)

target_link_libraries(
    sky360lib_demo_video
        PUBLIC
            "${OpenCV_LIBS}"
            sky360lib
)

set_target_properties(
    sky360lib_demo_video
        PROPERTIES
            FOLDER "apps"
)

install(
    TARGETS sky360lib_demo_video
    RUNTIME DESTINATION "bin"
    COMPONENT "apps"
)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_libraries(  # for <execution> stdlib
    sky360lib_demo_video
            PRIVATE
                TBB::tbb
    )
endif ()

# sky360lib_qhy_camera_loop
add_executable(
    sky360lib_qhy_camera_loop
        "src/qhy_camera_loop.cpp"
)

target_include_directories(
    sky360lib_qhy_camera_loop
        PUBLIC
            "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/api/include>"
)

target_link_libraries(
    sky360lib_qhy_camera_loop
        PUBLIC
            "${OpenCV_LIBS}"
            sky360lib
)

set_target_properties(
    sky360lib_qhy_camera_loop
        PROPERTIES
            FOLDER "apps"
)

install(
    TARGETS sky360lib_qhy_camera_loop
    RUNTIME DESTINATION "bin"
    COMPONENT "apps"
)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_libraries(  # for <execution> stdlib
    sky360lib_qhy_camera_loop
            PRIVATE
                TBB::tbb
    )
endif ()

set(VIDEOTEST ${CMAKE_SOURCE_DIR}/Dahua-20220901-184734.mp4)
if (EXISTS ${VIDEOTEST})
    message(STATUS "Copying Video Dahua-20220901-184734.mp4")
    configure_file(${VIDEOTEST} "${CMAKE_BINARY_DIR}/bin/Dahua-20220901-184734.mp4" COPYONLY)
endif ()
