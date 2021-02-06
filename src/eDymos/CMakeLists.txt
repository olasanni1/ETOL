cmake_minimum_required(VERSION 3.5)

project(eDymos)


if(PYTHON_EXECUTABLE)
  find_package(pybind11)

  if(pybind11_FOUND)
    add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}.cpp)

    target_link_libraries(${PROJECT_NAME}
      PUBLIC TrajectoryOptimizer pybind11::embed)
  else()
    message("Skipping eDymos...")
    message("To build this, install pybind11, openmdao, and dymos")
  endif()
else()
  message("Skipping eDymos...")
  message("To build this set, -DPYTHON_EXECUTABLE=</path/to/python3>")
endif()
