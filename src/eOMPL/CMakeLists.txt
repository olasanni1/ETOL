cmake_minimum_required(VERSION 2.8.9)

project(eOMPL)

find_package(ompl)

if(OMPL_FOUND)
  add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}.cpp)

  target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC TrajectoryOptimizer
    PRIVATE ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES} re2)

  target_include_directories(${PROJECT_NAME} PRIVATE ${OMPL_INCLUDE_DIRS})
else()
  message("Skipping eOMPL...")
  message("To build eOMPL, install OMPL")
  message("Try setting a OMPL_HOME environment variable")
endif()
