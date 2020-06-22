cmake_minimum_required(VERSION 2.8.9)

project(ePSOPT_Examples)

if(TARGET ePSOPT)
  find_package(PSOPT QUIET)

  set(PSOPT_EXAMPLES etol_psopt_example1)

  foreach(example ${PSOPT_EXAMPLES})
    add_executable(${example} ${example}.cpp)
    target_include_directories(${example} PRIVATE ${PSOPT_INCLUDE_DIRS})
    target_link_libraries(
      ${example}
      PUBLIC TrajectoryOptimizer ePSOPT
      PRIVATE ${PSOPT_LIBRARIES})
    target_compile_definitions(${example} PRIVATE ${PSOPT_DEFINITIONS})
  endforeach()
else()
  message("Skipping ePSOPT_Examples...")
  message("To build ePSOPT, create a PSOPT_HOME \
   environment variable")
  message("CXSPARSE and LAPACK must be installed")
  message("IPOPT_HOME, ADOLC_HOME and COLPACK_HOME \
    enrionment variables may be needed")
endif()
