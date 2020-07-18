cmake_minimum_required(VERSION 2.8.9)

project(ePSOPT)

find_package(PSOPT)

if(PSOPT_FOUND)
  add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}.cpp)

  target_link_libraries(${PROJECT_NAME}
    PUBLIC TrajectoryOptimizer ${PSOPT_LIBRARIES})

  target_include_directories(${PROJECT_NAME} PUBLIC ${PSOPT_INCLUDE_DIRS})

  target_compile_definitions(${PROJECT_NAME} PUBLIC ${PSOPT_DEFINITIONS})

else()
  message("Skipping ePSOPT...")
  message("To build this, create a PSOPT_HOME \
   environment variable")
  message("CXSPARSE and LAPACK must be installed")
  message("IPOPT_HOME, ADOLC_HOME and COLPACK_HOME \
    enrionment variables may be needed")
endif()
