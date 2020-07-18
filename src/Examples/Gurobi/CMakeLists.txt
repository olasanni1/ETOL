cmake_minimum_required(VERSION 2.8.9)

project(eGurobi_Examples)

if(TARGET eGurobi)
  set(GUROBI_EXAMPLES etol_gurobi_example1 etol_gurobi_example2
  etol_gurobi_example3 etol_gurobi_example4)

  foreach(example ${GUROBI_EXAMPLES})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example}
      PUBLIC eGurobi
      PRIVATE re2)
  endforeach()
else()
  message("Skipping eGurobi_Examples...")
  message("Requires eGurobi")
endif()
