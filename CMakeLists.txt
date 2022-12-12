cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(USE_ASAN)
    set(CMAKE_BUILD_TYPE Debug)
    add_compile_options(-fsanitize=address)
    add_link_options(-fsanitize=address)
endif()

add_compile_options(-std=c++17)
find_package(Boost REQUIRED COMPONENTS serialization)

find_package(Eigen3 REQUIRED)
ADD_DEFINITIONS(-DEIGEN_NO_DEBUG)
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(pybind11)
add_subdirectory(ompl)

include_directories(ompl/src ${CMAKE_CURRENT_BINARY_DIR}/ompl/src)
pybind11_add_module(_omplpy src/wrapper.cpp)
target_link_libraries(_omplpy PRIVATE ompl ${Boost_LIBRARIES})

add_executable(demo src/demo.cpp)
target_link_libraries(demo ompl ${Boost_LIBRARIES})
