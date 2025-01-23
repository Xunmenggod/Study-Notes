cmake_minimum_required(VERSION 3.2)
project(optimization)
set(CMAKE_CXX_STANDARD 17)

# include the thirdy party package
# find_package(Eigen3 REQUIRED)
# if (TARGET Eigen3::Eigen)
#     message(STATUS "----Found Eigen3 Package----")
# endif()
find_package(OsqpEigen REQUIRED)
if (TARGET OsqpEigen::OsqpEigen)
    message(STATUS "----Found OsqpEigen Package----")
endif()


# include_directories()

LINK_LIBRARIES(m)

add_executable(test
    test.cpp
)

target_link_libraries(test
    PUBLIC
    OsqpEigen::OsqpEigen
    m
)