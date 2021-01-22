cmake_minimum_required(VERSION 3.5)

project(eGurobi)

find_package(GUROBI)
find_package(TBB REQUIRED)
if(GUROBI_FOUND)
  add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}.cpp)

  target_link_libraries(${PROJECT_NAME}
    PUBLIC TrajectoryOptimizer
    PRIVATE ${GUROBI_LIBRARIES} tbb)

  target_include_directories(${PROJECT_NAME} PUBLIC ${GUROBI_INCLUDE_DIRS})
else()
  message("Skipping eGurobi...")
  message("To build eGurobi, install Gurobi")
  message("Try setting a GUROBI_HOME environment variable")
endif()
