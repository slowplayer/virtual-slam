cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE "Realease")
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

#opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#eigen
include_directories("/usr/include/eigen3")

#DBOW3
set(DBoW3_INCLUDE_DIRS "/usr/local/include")
set(DBoW3_LIBS "/usr/local/lib/libDBoW3.so")

#project-icp
add_executable(project-icp icp.cpp IcpSolver.cpp PointCloudGenerator.cpp)

#project-pnp
add_executable(project-pnp pnp.cpp PnpSolver.cpp PointCloudGenerator.cpp)

#project-bow
add_executable(project-bow bow.cpp BagOfWords.cpp)
target_link_libraries(project-bow ${OpenCV_LIBS} ${DBoW3_LIBS})