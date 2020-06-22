if(DEFINED ENV{GUROBI_HOME})
  set(GUROBI_ROOT_DIR "$ENV{GUROBI_HOME}")

  if(GUROBI_ROOT_DIR AND NOT GUROBI_FIND_QUIETLY)
    message("-- Looking for GUROBI and components in ${GUROBI_ROOT_DIR}")
  endif()
endif()

find_path(
  GUROBI_INCLUDE_DIR
  NAMES gurobi_c++.h
  HINTS ${GUROBI_ROOT_DIR}/include)

if(APPLE)
  find_library(
    GUROBI_LIBRARY
    NAMES libgurobi90.dylib libgurobi80.dylib
    HINTS ${GUROBI_ROOT_DIR}/lib)
  find_library(
    GUROBI_LIBRARY_API libgurobi_stdc++.a
    HINTS ${GUROBI_ROOT_DIR}/lib /opt/gurobi901/linux64/lib
          /opt/gurobi811/linux64/lib /opt/gurobi801/linux64/lib
          C:/gurobi901/win64/lib C:/gurobi801/win64/lib)
elseif(UNIX)
  find_library(
    GUROBI_LIBRARY
    NAMES libgurobi90.so libgurobi81.so libgurobi80.so
    HINTS ${GUROBI_ROOT_DIR}/lib /opt/gurobi901/linux64/lib
          /opt/gurobi811/linux64/lib /opt/gurobi801/linux64/lib)
  find_library(
    GUROBI_LIBRARY_API libgurobi_g++5.2.a
    HINTS ${GUROBI_ROOT_DIR}/lib /opt/gurobi901/linux64/lib
          /opt/gurobi811/linux64/lib /opt/gurobi801/linux64/lib
          C:/gurobi901/win64/lib C:/gurobi801/win64/lib)
elseif(WIN32)
  find_library(
    GUROBI_LIBRARY
    NAMES gurobi90.lib gurobi80.lib
    HINTS ${GUROBI_ROOT_DIR}/lib C:/gurobi901/win64/lib C:/gurobi801/win64/lib)
  find_library(
    GUROBI_LIBRARY_API libgurobi_c++.a
    HINTS ${GUROBI_ROOT_DIR}/lib /opt/gurobi901/linux64/lib
          /opt/gurobi811/linux64/lib /opt/gurobi801/linux64/lib
          C:/gurobi901/win64/lib C:/gurobi801/win64/lib)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GUROBI DEFAULT_MSG GUROBI_LIBRARY
                                  GUROBI_LIBRARY_API GUROBI_INCLUDE_DIR)

if(GUROBI_FOUND)
  if (NOT GUROBI_FIND_QUIETLY)
    message("—- Found GUROBI under ${GUROBI_INCLUDE_DIR}")
  endif()
  set(GUROBI_INCLUDE_DIRS ${GUROBI_INCLUDE_DIR})
  set(GUROBI_LIBRARIES ${GUROBI_LIBRARY_API} ${GUROBI_LIBRARY})
endif(GUROBI_FOUND)

mark_as_advanced(GUROBI_LIBRARY GUROBI_LIBRARY_API GUROBI_INCLUDE_DIR)
