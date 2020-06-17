# project
cmake_minimum_required (VERSION 2.6)
project (libviso2)

# directories
set (LIBVISO2_SRC_DIR src)

# include directory
include_directories("${LIBVISO2_SRC_DIR}")

# use sse3 instruction set
SET(CMAKE_CXX_FLAGS "-msse3")

# sources
FILE(GLOB LIBVISO2_SRC_FILES "src/*.cpp")

# make release version
set(CMAKE_BUILD_TYPE Release)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(viso2 SHARED
                src/filter.cpp
                src/matcher.cpp
                src/matrix.cpp
                src/reconstruction.cpp
                src/triangle.cpp
                src/viso.cpp
                src/viso_mono.cpp
                src/viso_stereo.cpp
)

# demo
add_executable(demo src/demo.cpp)
target_link_libraries(demo viso2 png)

