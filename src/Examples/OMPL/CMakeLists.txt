cmake_minimum_required(VERSION 3.5)

project(eOMPL_Examples)

if(TARGET eOMPL)
  set(OMPL_EXAMPLES etol_ompl_example1)

  foreach(example ${OMPL_EXAMPLES})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} PUBLIC eOMPL)
  endforeach()
else()
  message("Skipping eOMPL_Examples...")
  message("Requires eOMPL")
endif()
