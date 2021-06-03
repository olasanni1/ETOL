cmake_minimum_required(VERSION 3.5)
project(ETOL)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/../cmake")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set (BOOST_NO_SYSTEM_PATHS ON)
set (Boost_USE_MULTITHREADED ON)
set (Boost_DETAILED_FAILURE_MSG ON)
set (Boost_USE_STATIC_RUNTIME OFF)
set (Boost_USE_STATIC_LIBS OFF)
set (Boost_DEBUG OFF) # Turn on for debug info
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_definitions(-DPSTL_USE_PARALLEL_POLICIES)
add_definitions(-D__PSTL_USE_PAR_POLICIES)
add_definitions(-D__PSTL_CPP14_VARIABLE_TEMPLATES_PRESENT)

include_directories(${PROJECT_SOURCE_DIR}/../include)

SET (SUBDIRS TrajectoryOptimizer eGurobi ePSOPT eGLPK eOMPL eDymos eSCIP
             Examples)

option(DISABLE_GLPK "Disable GLPK interface" OFF)
option(DISABLE_GUROBI "Disable Gurobi interface" OFF)
option(DISABLE_SCIP "Disable SCIP interface" OFF)
option(DISABLE_PSOPT "Disable PSOPT interface" OFF)
option(DISABLE_DYMOS "Disable Dymos interface" OFF)
option(DISABLE_OMPL "Disable OMPL interface" OFF)
option(DISABLE_EXAMPLES "Disable ETOL examples" OFF)
FOREACH(subdir ${SUBDIRS})
  ADD_SUBDIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/${subdir})
ENDFOREACH()

option(BUILD_DOCS "Build documentation" OFF)
if (BUILD_DOCS)
	ADD_SUBDIRECTORY(docs)
endif()
