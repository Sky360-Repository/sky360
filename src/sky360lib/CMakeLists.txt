cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

list(APPEND CMAKE_MODULE_PATH
    "${CMAKE_SOURCE_DIR}/cmake/"
    "${CMAKE_SOURCE_DIR}/cmake/Modules/"
)
if ("${CMAKE_SOURCE_DIR}" STREQUAL "${CMAKE_BINARY_DIR}")
    message(FATAL_ERROR "In-source build not supported; generation should be done in a subdir, e.g.:  ${CMAKE_SOURCE_DIR}/build")
endif ()

#include(CheckLanguage)
include(FrameworkUtils)
#include(CheckFunctionExists)

project(sky360lib CXX)
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

set(SKY360_PYTHON_SUPPORT ON)

find_package(OpenCV 4.0 REQUIRED)
message(STATUS "Found OpenCV >=4.0")

if (USE_OPENMP)
    find_package(OpenMP REQUIRED)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

if (USE_FAST_MATH)
    add_definitions(-ffast-math)
endif ()
IF (WIN32)
    SET(CMAKE_CXX_FLAGS "-O2 -Ob2 -Oi -Ot -GL -Qpar -arch:AVX2 -fp:fast")
ENDIF()
IF (NOT WIN32)
    add_definitions(-O3)
    add_definitions(-march=native)
    add_definitions(-Wall)
    add_definitions(-Wno-deprecated)
    add_definitions(-Wextra)
    # add_definitions(-Wfatal-errors)
    # add_definitions(-Wshadow)
    # add_definitions(-Werror)
    # add_definitions(-pedantic-errors)
    add_definitions(-Wno-missing-braces)
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        # add_definitions(-Ofast)
        find_package(TBB REQUIRED)  # for <execution> stdlib
    endif ()
ENDIF()
if (USE_LINK_TIME_OPTIM)
    IF (NOT WIN32)
        add_definitions(-flto)
    ENDIF()
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

add_subdirectory(api)
add_subdirectory(apps)

if (SKY360_PYTHON_SUPPORT)
    message(STATUS "Building Python bindings lib")
    if (NOT DEFINED SKY360_PYTHON_VERSION)
        set(SKY360_PYTHON_VERSION 3)
    endif ()
    set(PYBIND11_LTO_CXX_FLAGS "")
    set(PYBIND11_PYTHON_VERSION ${SKY360_PYTHON_VERSION})

    add_subdirectory(modules/pybind11)
    message(STATUS "")
    message(STATUS "Python library status:")
    message(STATUS "    executable: ${PYTHON_EXECUTABLE}")
    message(STATUS "    library: ${PYTHON_LIBRARY}")
    message(STATUS "    include path: ${PYTHON_INCLUDE_DIRS}")

    if (NOT NUMPY_INCLUDE_DIR)
        exec_program ("${PYTHON_EXECUTABLE}"
            ARGS "-c \"import numpy; print(numpy.get_include())\""
            OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
            RETURN_VALUE NUMPY_NOT_FOUND)
    endif ()
    message(STATUS "NUMPY_INCLUDE_DIR: ${NUMPY_INCLUDE_DIR}\n")

    file(GLOB_RECURSE sky360_python_src wrapper/python/*.cpp)
    file(GLOB_RECURSE sky360_python_inc wrapper/python/*.h)
  
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/modules/pybind11/include)
    include_directories(${PYTHON_INCLUDE_DIRS})
    include_directories(${NUMPY_INCLUDE_DIR})

    pybind11_add_module(sky360_python ${sky360_python_src} ${sky360_python_inc})
    target_link_libraries(sky360_python PRIVATE sky360lib ${OpenCV_LIBS} ${PYTHON_LIBRARY} pybind11::module)
    set_target_properties(sky360_python PROPERTIES SUFFIX "${PYTHON_MODULE_EXTENSION}")
    target_compile_definitions(sky360_python PRIVATE SKY360_PYTHON_SUPPORT=1)
    set_property(TARGET sky360_python PROPERTY OUTPUT_NAME "pysky360")

    if (UNIX)
        execute_process(
            COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
                from distutils import sysconfig as sc
                print(sc.get_python_lib(prefix='', plat_specific=True))"
            OUTPUT_VARIABLE PYTHON_SITE
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        message(STATUS "")
        message(STATUS "The sky360 library python package will be installed at: ${PYTHON_SITE}\n")

        install(TARGETS sky360_python DESTINATION ${PYTHON_SITE})
    endif ()
endif ()
