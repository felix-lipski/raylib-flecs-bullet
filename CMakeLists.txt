cmake_minimum_required(VERSION 3.0)
project(Game)

add_executable(Game src/main.cpp)
target_include_directories(Game PRIVATE include)

# raylib-cpp
find_package(raylib QUIET)
if (NOT raylib_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG 4.5.0
    )
    FetchContent_MakeAvailable(raylib)
endif()
find_package(raylib_cpp QUIET)
if (NOT raylib_cpp_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        raylib_cpp
        GIT_REPOSITORY https://github.com/RobLoach/raylib-cpp.git
        GIT_TAG v4.5.1
    )
    FetchContent_MakeAvailable(raylib_cpp)
endif()
# set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 11)
target_link_libraries(${PROJECT_NAME} raylib raylib_cpp)

# bullet
target_include_directories(Game PRIVATE libs/bullet3/src)
add_subdirectory(libs/bullet3)
target_link_libraries(Game BulletDynamics BulletCollision LinearMath)

# flecs
add_subdirectory(libs/flecs)
target_link_libraries(${PROJECT_NAME} flecs::flecs_static)
