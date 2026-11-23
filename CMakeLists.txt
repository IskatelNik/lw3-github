cmake_minimum_required(VERSION 3.22)
project(LW3 LANGUAGES CXX)

set(SFML_STATIC_LIBRARIES ON)
find_package(SFML 3.1 CONFIG QUIET COMPONENTS Graphics
    HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../libs/sfml-install")
if(NOT TARGET SFML::Graphics)
    include(FetchContent)
    set(BUILD_SHARED_LIBS OFF)
    set(SFML_BUILD_AUDIO OFF)
    set(SFML_BUILD_NETWORK OFF)
    FetchContent_Declare(SFML
        GIT_REPOSITORY https://github.com/SFML/SFML.git
        GIT_TAG 3.1.0
        GIT_SHALLOW ON)
    FetchContent_MakeAvailable(SFML)
endif()

add_executable(LW3 main.cpp physics.hpp)
set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT LW3)
target_compile_features(LW3 PRIVATE cxx_std_20)
target_link_libraries(LW3 PRIVATE SFML::Graphics)
if(MSVC)
    target_compile_options(LW3 PRIVATE /W4 /utf-8)
endif()
