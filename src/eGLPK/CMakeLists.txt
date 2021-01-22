cmake_minimum_required(VERSION 3.5)

project(eGLPK)

find_package(GLPK)

if(GLPK_FOUND)
  add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}.cpp)

  target_link_libraries(${PROJECT_NAME}
    PUBLIC TrajectoryOptimizer
    PRIVATE re2 ${GLPK_LIBRARY})

  target_include_directories(${PROJECT_NAME} PUBLIC ${GLPK_INCLUDE_DIRS})
else()
  message("Skipping eGLPK...")
  message("To build eGLPK, install GLPK")
endif()
