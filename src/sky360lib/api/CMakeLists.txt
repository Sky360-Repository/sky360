cmake_minimum_required(VERSION 3.0)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories($ENV{QHYCCD_SDK_INCLUDE_DIRS})

if(MSVC)
  target_compile_options(sky360lib PRIVATE /EHsc)
endif()

file(GLOB_RECURSE LIB_SOURCES "*.cpp" "*.c")

add_library(sky360lib STATIC ${LIB_SOURCES})

target_link_libraries(sky360lib
                    PRIVATE
                        "${OpenCV_LIBS}"
                        qhyccd
                        )

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_link_libraries(sky360lib
            PRIVATE
                TBB::tbb
    )
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
    target_include_directories(sky360lib PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${_relative_dir}>
        $<INSTALL_INTERFACE:include/sky360lib/${_relative_dir}>
    )
endforeach()

install(TARGETS sky360lib
        EXPORT sky360libTargets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

foreach(_dir ${INCLUDE_DIRS})
    file(RELATIVE_PATH REL_DIR ${CMAKE_SOURCE_DIR} ${_dir})
    file(GLOB HEADER_FILES "${_dir}/*.h" "${_dir}/*.hpp")
    install(FILES ${HEADER_FILES} DESTINATION include/sky360lib/${REL_DIR})
endforeach()

install(EXPORT sky360libTargets
        FILE sky360libTargets.cmake
        DESTINATION lib/cmake/sky360lib
)

# Include the CMakePackageConfigHelpers module
include(CMakePackageConfigHelpers)

# Generate the config file that is includes the exports
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/sky360libConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/sky360libConfig.cmake"
    INSTALL_DESTINATION "lib/cmake/sky360lib"
)

# Generate the version file for the config file
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/sky360libConfigVersion.cmake"
    VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}"
    COMPATIBILITY AnyNewerVersion
)

# Install the configuration file
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/sky360libConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/sky360libConfigVersion.cmake
    DESTINATION lib/cmake/sky360lib
)

# Setup the exports for the build tree
export(EXPORT sky360libTargets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/sky360libTargets.cmake"
)

# specify that the library needs to be linked with the Boost libraries
target_link_libraries(sky360lib
                    PRIVATE
                        "${OpenCV_LIBS}"
                        qhyccd
                        )  