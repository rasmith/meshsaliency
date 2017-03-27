cmake_minimum_required(VERSION 2.8.12)
project(MeshSaliency)

# Set exe path.
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

# Set library path.`
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

# Set C++ flags.
set(CMAKE_CXX_FLAGS 
  "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations") 

# Set the module path (mainly for FindXXX.cmake files)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
											${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)


include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

# Find libigl.
find_package(LIBIGL REQUIRED)

# Set options for libigl.
option(LIBIGL_WITH_VIEWER      "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_OPENGL      "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW"           ON)
option(LIBIGL_WITH_PNG         "Use PNG"            ON)
option(LIBIGL_WITH_EMBREE      "Use Embree"         ON)

# Compile the options we need. 
add_subdirectory("${LIBIGL_INCLUDE_DIR}/../shared/cmake" "libigl")
message(STATUS "LIBIGL_INCLUDE_DIRS=${LIBIGL_INCLUDE_DIRS}")
include_directories(${LIBIGL_INCLUDE_DIRS})

# Add definitions.
add_definitions(${LIBIGL_DEFINITIONS})
message(STATUS "LIBIGL_DEFINITIONS=${LIBIGL_DEFINITIONS}")

# Find Eigen3.
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
message(STATUS "EIGEN3_INCLUDE_DIRS=${EIGEN3_INCLUDE_DIRS}")

message(STATUS "LIBIGL_PNG_EXTRA_LIBRARIES=${LIBIGL_PNG_EXTRA_LIBRARIES}")

# Compile and link render_views executable.
add_executable(render_views render_views.cc)
target_link_libraries(render_views ${LIBIGL_LIBRARIES}
                          ${LIBIGL_VIEWER_EXTRA_LIBRARIES}
                          ${LIBIGL_OPENGL_EXTRA_LIBRARIES}
                          ${LIBIGL_OPENGL_GLFW_EXTRA_LIBRARIES}
                          ${LIBIGL_PNG_EXTRA_LIBRARIES})

# Compile and link backproject_saliency executable.
add_executable(backproject_saliency backproject_saliency.cc)
target_compile_definitions(backproject_saliency PRIVATE ${LIBIGL_DEFINITIONS})
target_link_libraries(backproject_saliency ${LIBIGL_LIBRARIES}
                          ${LIBIGL_VIEWER_EXTRA_LIBRARIES}
                          ${LIBIGL_OPENGL_EXTRA_LIBRARIES}
                          ${LIBIGL_OPENGL_GLFW_EXTRA_LIBRARIES}
                          ${LIBIGL_PNG_EXTRA_LIBRARIES}
                          ${LIBIGL_EMBREE_EXTRA_LIBRARIES})

# Compile and link spectral_saliency executable.
add_executable(spectral_saliency spectral_saliency.cc)
target_link_libraries(spectral_saliency ${LIBIGL_LIBRARIES}
                          ${LIBIGL_VIEWER_EXTRA_LIBRARIES}
                          ${LIBIGL_OPENGL_EXTRA_LIBRARIES}
                          ${LIBIGL_OPENGL_GLFW_EXTRA_LIBRARIES}
                          ${LIBIGL_PNG_EXTRA_LIBRARIES})
