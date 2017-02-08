cmake_minimum_required(VERSION 2.8.12)
project(MeshSaliency)

# Set exe path.
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

# Set C++ flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 

# Set the module path (mainly for FindXXX.cmake files)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
											${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)

# Find libigl.
find_package(LIBIGL REQUIRED)
include_directories(${LIBIGL_INCLUDE_DIRS})

# Set options for libigl.
option(LIBIGL_WITH_VIEWER      "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_OPENGL      "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW"           ON)

# Compile the options we need. 
add_subdirectory("${LIBIGL_INCLUDE_DIR}/../shared/cmake" "libigl")

# Add definitions.
add_definitions(${LIBIGL_DEFINITIONS})
message(STATUS "LIBIGL_DEFINITIONS=${LIBIGL_DEFINITIONS}")

# Find Eigen3.
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
message(STATUS "EIGEN3_INCLUDE_DIRS=${EIGEN3_INCLUDE_DIRS}")

# Compile and link main.
add_executable(main main.cc)
target_link_libraries(main ${LIBIGL_LIBRARIES}
                           ${LIBIGL_VIEWER_EXTRA_LIBRARIES}
                           ${LIBIGL_OPENGL_EXTRA_LIBRARIES}
                           ${LIBIGL_OPENGL_GLFW_EXTRA_LIBRARIES})
