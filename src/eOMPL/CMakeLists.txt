cmake_minimum_required(VERSION 3.5)

project(eOMPL)

find_package(ompl)

if(OMPL_FOUND)
  add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}.cpp)

  target_link_libraries(${PROJECT_NAME}
    PUBLIC TrajectoryOptimizer ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES})

  target_include_directories(${PROJECT_NAME} PUBLIC ${OMPL_INCLUDE_DIRS})
  target_link_directories(${PROJECT_NAME} PUBLIC ${OMPL_LIBRARY_DIRS})
else()
  message("Skipping eOMPL...")
  message("To build eOMPL, install OMPL")
endif()
