cmake_minimum_required(VERSION 2.8.9)

project(eOMPL_Examples)

if(TARGET eOMPL)
  find_package(ompl QUIET)

  set(OMPL_EXAMPLES etol_ompl_example1)

  foreach(example ${OMPL_EXAMPLES})
    add_executable(${example} ${example}.cpp)
    target_include_directories(${example} PRIVATE ${OMPL_INCLUDE_DIRS})
    target_link_libraries(${example} PRIVATE TrajectoryOptimizer eOMPL
    ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES} re2)
  endforeach()
else()
  message("Skipping eOMPL_Examples...")
  message(
    "To build eOMPL, create a OMPL_HOME \
    environment variable or install OMPL"
  )
endif()
