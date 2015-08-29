cmake_minimum_required(VERSION 2.8)

#Configuration du projet
project(meng_project)

find_package(OpenCV 2.4.12 EXACT REQUIRED)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

set(CMAKE_CXX_FLAGS "-std=c++11 -ggdb3 -Wall")

add_subdirectory(src/core)

#Configuration de l'exécutable
file(
GLOB_RECURSE
source_files
src/main.cpp
)

file(
GLOB_RECURSE
source_files_test
src/test/*
)

include_directories(src/)
include_directories(src/core/)
include_directories(src/test)

add_executable(
meng_project
${source_files}
$<TARGET_OBJECTS:core>
)

add_executable(
model
src/model.cpp
)

add_executable(
test
${source_files_test}
$<TARGET_OBJECTS:core>
)

target_link_libraries(
meng_project
${OpenCV_LIBS}
opengv
gsl
gslcblas
lbfgs
pthread
)

target_link_libraries(
model
${OpenCV_LIBS}
)

target_link_libraries(
test
${OpenCV_LIBS}
opengv
gsl
gslcblas
lbfgs
pthread
)

