cmake_minimum_required(VERSION 3.2)
project(CPP)
set(CXX_STANDARD 17)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include()
)

aux_source_directory(${PROJECT_SOURCE_DIR}/src SOURCE_LISTS)

# add_executable(hello ${SOURCE_LISTS})
add_executable(time ${PROJECT_SOURCE_DIR}/src/time.cpp)
