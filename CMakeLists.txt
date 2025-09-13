# ------------------------------------------------------------------------------
# FWR-Cam Library
# Copyright (c) 2025 Framework Robotics GmbH
# 
# Authors: Michael Lampert
# 
# Licensed under the BSD 3-Clause License.
# You may use, modify, and redistribute this file under those terms.
# 
# See the LICENSE file in the project root for full license information.
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.19)

# === Project Information & Licensing ===
message(STATUS "FWR-Cam_lnx is open-source software. See LICENSE file for details.")

# Prevent in-source builds.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "In-source builds are not allowed. Create a separate build directory and run cmake there:
   mkdir build && cd build
   cmake ..")
endif()

# Set default install prefix if not provided.
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "Default install dir" FORCE)
endif()

# === Read and Parse Version Information ===
file(READ "${CMAKE_SOURCE_DIR}/VERSION" PROJECT_VERSION_STRING)
string(STRIP "${PROJECT_VERSION_STRING}" PROJECT_VERSION_STRING)
string(REGEX MATCH "^[0-9]+\\.[0-9]+\\.[0-9]+\\.?[0-9]*" PROJECT_VERSION "${PROJECT_VERSION_STRING}")
string(REGEX MATCH "_[a-zA-Z0-9]+" PROJECT_VERSION_SUFFIX "${PROJECT_VERSION_STRING}")
string(REPLACE "_" "" PROJECT_VERSION_SUFFIX "${PROJECT_VERSION_SUFFIX}")

message(STATUS "Project version: ${PROJECT_VERSION}")
message(STATUS "Version suffix: ${PROJECT_VERSION_SUFFIX}")

project(FWR_Cam_lnx VERSION ${PROJECT_VERSION} LANGUAGES CXX)

# Cache version information internally.
set(FWR_Cam_lnx_VERSION ${PROJECT_VERSION} CACHE INTERNAL "FWR_Cam_lnx version")
set(FWR_Cam_lnx_VERSION_SUFFIX ${PROJECT_VERSION_SUFFIX} CACHE INTERNAL "FWR_Cam_lnx version suffix")

# === Version Header Configuration ===
if(PROJECT_VERSION_SUFFIX STREQUAL "")
    set(FWR_FPBASE_FULL_VERSION ${PROJECT_VERSION})
    set(CPACK_FULL_VERSION ${PROJECT_VERSION})
else()
    set(FWR_FPBASE_FULL_VERSION "${PROJECT_VERSION}-${PROJECT_VERSION_SUFFIX}")
    set(CPACK_FULL_VERSION "${PROJECT_VERSION}_${PROJECT_VERSION_SUFFIX}")
endif()

configure_file(
    "${CMAKE_SOURCE_DIR}/include/FWR-Cam_lnx/Version.hpp.in"
    "${CMAKE_SOURCE_DIR}/include/FWR-Cam_lnx/Version.gen.hpp"
    @ONLY
)

# === Build Options ===
# BUILD_SHARED_LIBS is built-in: if ON, builds a shared library.
if(BUILD_SHARED_LIBS)
    message(STATUS "Building as a shared library")
    set(LIB_TYPE SHARED)
else()
    message(STATUS "Building as a static library")
    set(LIB_TYPE STATIC)
endif()

# Locate the Linux system header videodev2.h.
find_path(VIDEODEV2_HEADER_PATH
    NAMES videodev2.h
    PATHS ${CMAKE_SYSTEM_INCLUDE_PATH}
    PATH_SUFFIXES linux
)
if(NOT VIDEODEV2_HEADER_PATH)
    message(FATAL_ERROR "Could not locate videodev2.h. Ensure Linux kernel headers are installed.")
else()
    set(VIDEODEV2_HEADER "${VIDEODEV2_HEADER_PATH}/videodev2.h")
    message(STATUS "Found videodev2.h: ${VIDEODEV2_HEADER}")
endif()

# === Source File Collection ===
file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/source/FWR-Cam_lnx/*.cpp")

# === Library Target Definition ===
add_library(FWR_Cam_lnxLib ${LIB_TYPE} ${SOURCES})
# add_dependencies(FWR_Cam_lnxLib GenerateFourCCHeader)

string(REPLACE "." ";" VERSION_PARTS "${PROJECT_VERSION}")
list(GET VERSION_PARTS 0 PROJECT_VERSION_MAJOR)
list(GET VERSION_PARTS 1 PROJECT_VERSION_MINOR)
list(GET VERSION_PARTS 2 PROJECT_VERSION_PATCH)

# Define SOVERSION scheme:
# - If major == 0 → soversion = major.minor
# - Else           → soversion = major
if(PROJECT_VERSION_MAJOR EQUAL 0)
    set(FWR_Cam_lnx_SOVERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
else()
    set(FWR_Cam_lnx_SOVERSION "${PROJECT_VERSION_MAJOR}")
endif()


set_target_properties(FWR_Cam_lnxLib PROPERTIES
    OUTPUT_NAME "FWR-Cam_lnx"
    VERSION "${PROJECT_VERSION}"
    SOVERSION "${FWR_Cam_lnx_SOVERSION}"
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    LICENSE "Open source, 3-clause BSD - see LICENSE file"
)

if(BUILD_SHARED_LIBS)
    set_target_properties(FWR_Cam_lnxLib PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

target_compile_options(FWR_Cam_lnxLib PRIVATE
    -Wpedantic -Wextra -Wall -Wunreachable-code -Wreturn-type -Werror
    -ftemplate-backtrace-limit=0
    -fconcepts-diagnostics-depth=10 -fdiagnostics-show-template-tree -fpretty-templates
)

target_include_directories(FWR_Cam_lnxLib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/third_party/magic_enum/include>
)

find_library(LIBUSB_LIB usb-1.0 REQUIRED)
find_library(LIBUDEV_LIB udev REQUIRED)

target_link_libraries(FWR_Cam_lnxLib
    ${LIBUSB_LIB}
    ${LIBUDEV_LIB}
)

# === Installation Rules ===
install(TARGETS FWR_Cam_lnxLib
    EXPORT FWR_Cam_lnxTargets
    ARCHIVE DESTINATION lib/FWR-Cam_lnx
    LIBRARY DESTINATION lib/FWR-Cam_lnx
    RUNTIME DESTINATION bin/FWR-Cam_lnx
)

configure_file(
    "${CMAKE_SOURCE_DIR}/cmake/FWR-Cam_lnx.pc.in"
    "${CMAKE_BINARY_DIR}/FWR-Cam_lnx.pc"
    @ONLY
)
install(FILES "${CMAKE_BINARY_DIR}/FWR-Cam_lnx.pc"
    DESTINATION lib/pkgconfig
)

# Install public headers.
install(DIRECTORY "${CMAKE_SOURCE_DIR}/include/FWR-Cam_lnx/"
    DESTINATION include/FWR-Cam_lnx
    FILES_MATCHING PATTERN "*.hpp" PATTERN "*.tpp"
)

install(FILES
    "${CMAKE_SOURCE_DIR}/include/FWR-Cam_lnx/Version.gen.hpp"
    DESTINATION include/FWR-Cam_lnx
)

# Export targets for downstream use.
install(EXPORT FWR_Cam_lnxTargets
    FILE FWR_Cam_lnxTargets.cmake
    NAMESPACE FWR_Cam_lnx::
    DESTINATION lib/cmake/FWR_Cam_lnx
)

# Install the package configuration files.
include(CMakePackageConfigHelpers)
configure_package_config_file(
    "${CMAKE_SOURCE_DIR}/cmake/FWR_Cam_lnxConfig.cmake.in"
    "${CMAKE_BINARY_DIR}/FWR_Cam_lnxConfig.cmake"
    INSTALL_DESTINATION lib/cmake/FWR_Cam_lnx
)
install(FILES
    "${CMAKE_BINARY_DIR}/FWR_Cam_lnxConfig.cmake"
    DESTINATION lib/cmake/FWR_Cam_lnx
)
write_basic_package_version_file(
    "${CMAKE_BINARY_DIR}/FWR_Cam_lnxConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
install(FILES "${CMAKE_BINARY_DIR}/FWR_Cam_lnxConfigVersion.cmake"
    DESTINATION lib/cmake/FWR_Cam_lnx
)

# Install additional documentation and license files.
install(FILES
    "${CMAKE_SOURCE_DIR}/LICENSE"
    "${CMAKE_SOURCE_DIR}/NOTICE"
    "${CMAKE_SOURCE_DIR}/README.md"
    "${CMAKE_SOURCE_DIR}/VERSION"
    "${CMAKE_SOURCE_DIR}/AUTHORS"
    DESTINATION share/FWR-Cam_lnx
)


# --- Packaging Configuration ---
set(CPACK_PACKAGE_NAME "FWR-Cam_lnx")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_CONTACT "info@fw-robotics.de")
# List the documentation files that you want in the package.
# set(CPACK_SOURCE_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}")
set(CPACK_SOURCE_IGNORE_FILES "/build/;/\\\\.git/;/\\\\.github/")

# Use the full version with underscore in the package file name.
set(CPACK_PACKAGE_FILE_NAME "FWR-Cam_lnx-${CPACK_FULL_VERSION}-Linux")

# Configure to create a tar.gz archive. Adjust the generator if needed.
set(CPACK_GENERATOR "TGZ")
set(CPACK_OUTPUT_FILE_PREFIX "${CMAKE_SOURCE_DIR}/distrib")

include(CPack)
