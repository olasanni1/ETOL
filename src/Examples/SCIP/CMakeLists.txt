cmake_minimum_required(VERSION 2.8.9)

project(eSCIP_Examples)

if(TARGET eSCIP)
  set(SCIP_EXAMPLES etol_scip_example1)

  foreach(example ${SCIP_EXAMPLES})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example}
      PUBLIC eSCIP
      PRIVATE re2)
  endforeach()
else()
  message("Skipping eSCIP_Examples...")
  message("Requires eSCIP")
endif()
