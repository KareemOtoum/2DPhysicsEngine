cmake_minimum_required(VERSION 3.10)
project(2DPhysicsEngine)

add_subdirectory(external/glfw)

file(GLOB SOURCES
    src/collision.cpp
    src/glad.c
    src/main.cpp
    src/RigidBody.cpp
    src/visuals.cpp
    src/world.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE include)

target_link_libraries(${PROJECT_NAME} glfw)
