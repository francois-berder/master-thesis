cmake_minimum_required(VERSION 2.8)

project(core)

file(
GLOB
source_files
*.cpp
*.hpp
)

add_library(core OBJECT ${source_files})

