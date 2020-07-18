cmake_minimum_required(VERSION 2.8.9)

project(eGLPK_Examples)

if(TARGET eGLPK)
  set(GLPK_EXAMPLES etol_glpk_example1)

  foreach(example ${GLPK_EXAMPLES})
    add_executable(${example} ${example}.cpp)
    target_link_libraries(${example} PUBLIC eGLPK)

  endforeach()
else()
  message("Skipping eGLPK_Examples...")
  message("Requires eGLPK")
endif()
