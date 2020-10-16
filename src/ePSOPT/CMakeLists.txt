cmake_minimum_required(VERSION 2.8.9)

project(ePSOPT)

find_package(PSOPT)

if(PSOPT_FOUND)
  find_package(Eigen3 REQUIRED NO_MODULE)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(adolc REQUIRED IMPORTED_TARGET adolc)
  pkg_check_modules(ipopt REQUIRED IMPORTED_TARGET ipopt)
  pkg_check_modules(snopt7 IMPORTED_TARGET snopt7)
  
  add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}.cpp)

  target_link_libraries(${PROJECT_NAME} 
    PUBLIC TrajectoryOptimizer
    PRIVATE PSOPT PkgConfig::adolc PkgConfig::ipopt Eigen3::Eigen)
  
  if(snopt7_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE gfortran PkgConfig::snopt7) 
  endif()
    
else()
  message("Skipping ePSOPT...")
  message("Requires: ADOLC, IPOPT, and Eigen3")
  message("Optional: SNOPT")
endif()
