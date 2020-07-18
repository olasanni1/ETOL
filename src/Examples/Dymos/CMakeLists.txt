cmake_minimum_required(VERSION 2.8.9)

project(eDymos_Examples)

if(TARGET eDymos)
  set(Dymos_EXAMPLES etol_dymos_example1)

  foreach(example ${Dymos_EXAMPLES})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} PUBLIC eDymos)
  endforeach()
else()
  message("Skipping eDymos_Examples...")
  message("Requires eDymos")
endif()
