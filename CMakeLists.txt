cmake_minimum_required(VERSION 3.5)
project(rerun)

# set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O2") 
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

#========================
#  common dependencies
#========================
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(rerun_sdk REQUIRED)
find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


#========================
#  dora
#========================
set(DORA_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/dora/include)

set(DORA_NODE_API_LIB 
${CMAKE_SOURCE_DIR}/dora/lib/libdora_node_api_c.a
    CACHE FILEPATH "Path to libdora_node_api_c.a"
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${DORA_INCLUDE_DIR}
  ${COMMON_INCLUDE_DIR}
  ./include
)

add_executable(rerun src/rerun.cpp)
target_link_libraries(truckrerun PRIVATE 
  ${PCL_LIBRARIES}
  ${DORA_NODE_API_LIB}
  rerun_sdk
  m
  rt
  dl
)