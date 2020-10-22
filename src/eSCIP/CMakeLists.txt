cmake_minimum_required(VERSION 2.8.9)

project(eSCIP)

find_package(SCIP)

if(SCIP_FOUND)
  add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}.cpp)

  target_link_libraries(${PROJECT_NAME}
    PUBLIC TrajectoryOptimizer
    PRIVATE re2 ${SCIP_LIBRARIES})

  target_include_directories(${PROJECT_NAME}
    PUBLIC ${SCIP_INCLUDE_DIRS} ${LIBM})
else()
  message("Skipping eSCIP...")
  message("To build eSCIP, install SCIP")
endif()
