cmake_minimum_required(VERSION 3.1)
project(eigen_state_space_systems)

if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS eigen_matrix_utils)
find_package(Eigen3 REQUIRED COMPONENTS Core Dense)

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS eigen_matrix_utils
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(test_state_space_matrix src/test_state_space_matrix.cpp)
add_dependencies(test_state_space_matrix ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_state_space_matrix
                      ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
)

add_executable(test_filters src/test_filters.cpp)
add_dependencies(test_filters ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_filters
#                       ${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
)


add_executable(        test_controllers src/test_pid.cpp)
add_dependencies(      test_controllers ${catkin_EXPORTED_TARGETS})
target_link_libraries( test_controllers
#                       ${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(filtered_values_test test/filtered_values_test.launch test/filtered_values_test.cpp)
  target_link_libraries(filtered_values_test ${catkin_LIBRARIES} ${rostest_LIBRARIES} )

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME filtered_values_test_coverage_report
      DEPENDENCIES filtered_values_test
    )
  endif()
endif()

