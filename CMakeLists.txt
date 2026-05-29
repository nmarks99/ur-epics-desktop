cmake_minimum_required (VERSION 3.22)

project(ur-epics-desktop)

# Generate compile_commands.json for LSP's
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# C++ Standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build options
option(FETCH_RAYLIB "Fetch raylib using FetchContent" ON)
option(FETCH_IMGUI "Fetch ImGui using FetchContent" ON)
option(FETCH_RLIMGUI "Fetch rlImGui using FetchContent" ON)
option(EPICS_STATIC "Link EPICS libraries statically" OFF)

# --- EPICS Base ---------------------------------------------------------------
if(NOT DEFINED EPICS_BASE OR EPICS_BASE STREQUAL "")
    set(EPICS_BASE $ENV{EPICS_BASE})
endif()
if(NOT EPICS_BASE OR EPICS_BASE STREQUAL "")
    message(FATAL_ERROR "Missing required variable 'EPICS_BASE'. Provide it via -DEPICS_BASE=... or set the EPICS_BASE environment variable.")
endif()
if(NOT IS_DIRECTORY "${EPICS_BASE}")
    message(FATAL_ERROR "Specified EPICS_BASE does not exist: ${EPICS_BASE}")
endif()

set(EPICS_BASE "${EPICS_BASE}" CACHE STRING "Path to EPICS base")
message(STATUS "Found EPICS_BASE: ${EPICS_BASE}")

set(EPICS_HOST_ARCH "$ENV{EPICS_HOST_ARCH}" CACHE STRING "Host architecture (e.g. linux-x86_64)")
if(NOT EPICS_HOST_ARCH)
    message(FATAL_ERROR "'EPICS_HOST_ARCH' not defined")
endif()
message(STATUS "EPICS_HOST_ARCH: ${EPICS_HOST_ARCH}")

set(EPICS_LIB_DIR "${EPICS_BASE}/lib/${EPICS_HOST_ARCH}")
if(NOT DEFINED CMAKE_INSTALL_RPATH AND NOT EPICS_STATIC)
    set(CMAKE_INSTALL_RPATH "${EPICS_LIB_DIR}")
endif()
if(EPICS_STATIC)
    set(CMAKE_FIND_LIBRARY_SUFFIXES .a)
endif()
foreach(LIB pvAccessCA ca pvAccess pvData Com)
    find_library(EPICS_${LIB} ${LIB}
        PATHS ${EPICS_LIB_DIR}
        NO_DEFAULT_PATH
        REQUIRED
    )
    message(STATUS "Found EPICS ${LIB}: ${EPICS_${LIB}}")
endforeach()
set(EPICS_BASE_LIBRARIES
    ${EPICS_pvAccessCA}
    ${EPICS_ca}
    ${EPICS_pvAccess}
    ${EPICS_pvData}
    ${EPICS_Com}
)
if(EPICS_STATIC)
    list(APPEND EPICS_BASE_LIBRARIES readline dl rt pthread)
endif()

set(POTENTIAL_EPICS_INCLUDES
    ${EPICS_BASE}/include
    ${EPICS_BASE}/include/compiler/gcc
    ${EPICS_BASE}/include/compiler/clang
    ${EPICS_BASE}/include/pv
    ${EPICS_BASE}/include/pva
    ${EPICS_BASE}/include/os/Linux
    ${EPICS_BASE}/include/os/Darwin
)

set(EPICS_BASE_INCLUDES "")
foreach(DIR ${POTENTIAL_EPICS_INCLUDES})
    if(IS_DIRECTORY "${DIR}")
        list(APPEND EPICS_BASE_INCLUDES "${DIR}")
    endif()
endforeach()
# ------------------------------------------------------------------------------

# FetchContent for getting dependencies
if (FETCH_RAYLIB OR FETCH_IMGUI OR FETCH_RLIMGUI)
    include(FetchContent)
    set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
    set(FETCHCONTENT_QUIET OFF)
endif()

# Clone raylib from GitHub, or find it on the system
if (FETCH_RAYLIB)
    message(STATUS "Cloning raylib using FetchContent.")
    FetchContent_Declare(raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG         6.0
        GIT_SHALLOW     TRUE
        GIT_PROGRESS    TRUE
    )
    FetchContent_MakeAvailable(raylib)
else()
    message(STATUS "Attempting to find raylib on the system.")
    find_package(raylib REQUIRED STATIC CONFIG HINTS ${raylib_DIR})
    if(raylib_FOUND)
        message(STATUS "raylib found!")
        message(STATUS "raylib DIR: ${raylib_DIR}")
    else()
        message(FATAL_ERROR "raylib not found on your system. Please install it or enable -DFETCH_RAYLIB=ON.")
    endif()
endif()

# Fetch ImGui
if (FETCH_IMGUI)
    FetchContent_Declare(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG         docking
        GIT_SHALLOW     TRUE
        GIT_PROGRESS    TRUE
    )
    FetchContent_MakeAvailable(imgui)

    # Define imgui library manually since it lacks a root CMakeLists.txt
    add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    )

    target_include_directories(imgui PUBLIC
        ${imgui_SOURCE_DIR}
    )
endif()

# Clone rlImGui from GitHub, or find it on the system
if (FETCH_RLIMGUI)
    message(STATUS "Cloning rlImGui using FetchContent.")
    FetchContent_Declare(rlimgui
        GIT_REPOSITORY https://github.com/raylib-extras/rlImGui
        GIT_TAG         main
        GIT_SHALLOW     TRUE
        GIT_PROGRESS    TRUE
    )
    FetchContent_MakeAvailable(rlimgui)
else()
    message(STATUS "Attempting to find rlimgui on the system.")
    find_package(rlimgui REQUIRED STATIC CONFIG HINTS ${rlimgui_DIR})
    if(rlimgui_FOUND)
        message(STATUS "rlimgui found!")
        message(STATUS "rlimgui DIR: ${rlimgui_DIR}")
    else()
        message(FATAL_ERROR "rlimgui not found on your system. Please install it or enable -DFETCH_RLIMGUI=ON.")
    endif()
endif()

#####################################################
add_executable(ur-epics-desktop
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ur.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rl_utils.cpp
    # ${CMAKE_CURRENT_SOURCE_DIR}/rlImGui/rlImGui.cpp
    ${rlimgui_SOURCE_DIR}/rlImGui.cpp
)

target_compile_definitions(ur-epics-desktop PRIVATE
    UR_RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources"
)

target_include_directories(ur-epics-desktop PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    # ${CMAKE_CURRENT_SOURCE_DIR}/rlImGui
    ${rlimgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}
    ${EPICS_BASE_INCLUDES}
)

target_link_libraries(ur-epics-desktop PRIVATE
    raylib
    imgui
    ${EPICS_BASE_LIBRARIES}
)
