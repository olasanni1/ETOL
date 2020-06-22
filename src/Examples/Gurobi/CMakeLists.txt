cmake_minimum_required(VERSION 2.8.9)

project(eGurobi_Examples)

if(TARGET eGurobi)
  find_package(GUROBI QUIET)

  set(GUROBI_EXAMPLES etol_gurobi_example1 etol_gurobi_example2
  etol_gurobi_example3 etol_gurobi_example4)

  foreach(example ${GUROBI_EXAMPLES})
    add_executable(${example} ${example}.cpp)
    target_include_directories(${example} PRIVATE ${GUROBI_INCLUDE_DIRS})
    target_link_libraries(${example} PRIVATE TrajectoryOptimizer
    ${GUROBI_LIBRARIES} eGurobi re2)
  endforeach()
else()
  message("Skipping eGurobi_Examples...")
  message(
    "To build eGurobi, create a GUROBI_HOME \
    environment variable or install Gurobi"
  )
endif()
