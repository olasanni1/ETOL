cmake_minimum_required(VERSION 3.5)

project(eGLPK)

if (DISABLE_GLPK)
  return()
endif()

find_package(GLPK)

if(GLPK_FOUND)
  add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}.cpp)

  target_link_libraries(${PROJECT_NAME}
    PUBLIC TrajectoryOptimizer ${GLPK_LIBRARY})

  target_include_directories(${PROJECT_NAME} PUBLIC ${GLPK_INCLUDE_DIRS})
else()
  message("Skipping eGLPK...")
  message("To build eGLPK, install GLPK")
endif()
