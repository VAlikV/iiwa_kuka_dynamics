cmake_minimum_required(VERSION 3.5.0)

set(PROJECT Dynamics)

project(${PROJECT} VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Threads REQUIRED)
# find_library(OpenBLAS_LIB libopenblas NAMES libopenblas openblas HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
# find_library(Lapack_LIB liblapack NAMES liblapack lapack HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
# find_library(FORTRUN libgfortran.so.5 NAMES libgfortran5 gfortran gfortran5  gfortran REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)
# find_library(Pinocchio_LIB libpinocchio_default NAMES libpinocchio_default.so.3.3.0 libpinocchio pinocchio HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
# find_library(Pinocchio_Parser_LIB libpinocchio_parsers NAMES libpinocchio_parsers.so.3.3.0 libpinocchio pinocchio HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
# find_library(FORTRUN libgfortran.so.5 NAMES libgfortran5 gfortran gfortran5 gfortran REQUIRED)

set(USABLE_FUNCTIONS ${CMAKE_CURRENT_SOURCE_DIR}/usable_functions)

add_executable(${PROJECT}
    main.cpp
    
    # ${USABLE_FUNCTIONS}/usable_functions.hpp    ${USABLE_FUNCTIONS}/usable_functions.cpp
)

target_link_libraries(${PROJECT} 
    # ${OpenBLAS_LIB}
    # ${Lapack_LIB}
    # ${FORTRUN}
    Eigen3::Eigen
    Threads::Threads
    pinocchio::pinocchio
    # ${Pinocchio_LIB}
    # ${Pinocchio_Parser_LIB}
)

# target_include_directories(${PROJECT} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)