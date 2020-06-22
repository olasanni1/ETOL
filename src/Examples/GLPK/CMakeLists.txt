cmake_minimum_required(VERSION 2.8.9)

project(eGLPK_Examples)

if(TARGET eGLPK)
  find_package(GLPK QUIET)

  set(GLPK_EXAMPLES etol_glpk_example1)

  foreach(example ${GLPK_EXAMPLES})
    add_executable(${example} ${example}.cpp)
    target_include_directories(${example} PRIVATE ${GLPK_INCLUDE_DIRS})
    target_link_libraries(${example} PRIVATE TrajectoryOptimizer eGLPK
                                             ${GLPK_LIBRARIES} re2)
  endforeach()
else()
  message("Skipping eGLPK_Examples...")
  message(
    "To build eGLPK, create a GLPK_HOME \
    environment variable or install GLPK"
  )
endif()
