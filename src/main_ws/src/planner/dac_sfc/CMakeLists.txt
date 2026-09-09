cmake_minimum_required(VERSION 2.8.3)
project(dac_sfc)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  plan_env
  path_searching
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dac_sfc_core
  CATKIN_DEPENDS roscpp plan_env path_searching
)

include_directories(
  include
  include/dac_sfc/upstream
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(dac_sfc_core
  src/dac_sfc_engine.cpp
  src/dac_route_adapter.cpp
  src/deployment_logger.cpp
)
target_link_libraries(dac_sfc_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_executable(dac_sfc_engine_self_test test/test_engine.cpp)
  target_link_libraries(dac_sfc_engine_self_test dac_sfc_core ${catkin_LIBRARIES})
  add_test(NAME dac_sfc_engine_self_test COMMAND dac_sfc_engine_self_test)
endif()

install(TARGETS dac_sfc_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
