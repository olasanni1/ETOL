cmake_minimum_required(VERSION 3.5)

project(ePSOPT)

if(DISABLE_PSOPT)
  return()
endif()

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
           PSOPT PkgConfig::adolc PkgConfig::ipopt Eigen3::Eigen)

  if(snopt7_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC gfortran PkgConfig::snopt7) 
  endif()
  
  if(OpenMP_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC OpenMP::OpenMP_CXX)
  endif()  
else()
  message("Skipping ePSOPT...")
  message("Requires: ADOLC, IPOPT, and Eigen3")
  message("Optional: SNOPT, and OpenMP")
endif()
