cmake_minimum_required(VERSION 2.8.9)

project(eGLPK)

find_package(GLPK)

if(GLPK_FOUND)
  add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}.cpp)

  target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC TrajectoryOptimizer
    PRIVATE ${GLPK_LIBRARY} re2)

  target_include_directories(${PROJECT_NAME} PRIVATE ${GLPK_INCLUDE_DIRS})
else()
  message("Skipping eGLPK...")
  message("To build eGLPK, install GLPK")
  message("Try setting a GLPK_HOME environment variable")
endif()
