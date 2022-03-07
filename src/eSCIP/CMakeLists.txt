cmake_minimum_required(VERSION 3.5)

project(eSCIP)

if(DISABLE_SCIP)
  return()
endif()

find_package(SCIP)
find_package(TBB)

if(SCIP_FOUND)
  add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}.cpp)

  target_link_libraries(${PROJECT_NAME}
    PUBLIC TrajectoryOptimizer ${SCIP_LIBRARIES}
           $<IF:$<BOOL:${TBB_FOUND}>,TBB::tbb,tbb>)

  target_include_directories(${PROJECT_NAME}
    PUBLIC ${SCIP_INCLUDE_DIRS} ${LIBM})
else()
  message("Skipping eSCIP...")
  message("To build eSCIP, install SCIP")
endif()
