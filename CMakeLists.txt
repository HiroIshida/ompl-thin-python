cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-fsanitize=address)
add_link_options(-fsanitize=address)

add_compile_options(-std=c++17)
find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)

find_package(Eigen3 REQUIRED)
ADD_DEFINITIONS(-DEIGEN_NO_DEBUG)
include_directories(${EIGEN3_INCLUDE_DIR})

add_subdirectory(pybind11)
include_directories(${OMPL_INCLUDE_DIRS})
pybind11_add_module(_omplpy wrapper.cpp)
target_link_libraries(_omplpy PRIVATE ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(main main.cpp)
#target_link_libraries(main ${OMPL_LIBRARIES} ${Boost_LIBRARIES})
