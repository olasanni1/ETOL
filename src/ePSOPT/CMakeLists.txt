cmake_minimum_required(VERSION 3.5)

project(ePSOPT)

find_package(PSOPT)

if(PSOPT_FOUND)
  find_package(Eigen3 REQUIRED NO_MODULE)
  find_package(PkgConfig REQUIRED)
  find_package(OpenMP)
  find_package(TBB)
  pkg_check_modules(adolc REQUIRED IMPORTED_TARGET adolc)
  pkg_check_modules(ipopt REQUIRED IMPORTED_TARGET ipopt)
  pkg_check_modules(snopt7 IMPORTED_TARGET snopt7)
  
  add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}.cpp)

  target_link_libraries(${PROJECT_NAME} 
  PUBLIC TrajectoryOptimizer $<IF:$<BOOL:${TBB_FOUND}>,TBB::tbb,tbb> 
  PRIVATE PSOPT PkgConfig::adolc PkgConfig::ipopt Eigen3::Eigen)
  if(snopt7_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE gfortran PkgConfig::snopt7) 
  endif()
  
  if(OpenMP_FOUND)
    set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS
      ${OpenMP_CXX_FLAGS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenMP_C_FLAGS})
  endif()  
else()
  message("Skipping ePSOPT...")
  message("Requires: ADOLC, IPOPT, and Eigen3")
  message("Optional: SNOPT, and OpenMP")
endif()
