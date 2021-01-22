cmake_minimum_required(VERSION 3.5)

project(ePSOPT_Examples)

if(TARGET ePSOPT)
  set(PSOPT_EXAMPLES etol_psopt_example1)

  foreach(example ${PSOPT_EXAMPLES})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} PUBLIC ePSOPT)
  endforeach()
else()
  message("Skipping ePSOPT_Examples...")
  message("Requires ePSOPT")
endif()
