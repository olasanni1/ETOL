# Modified file from ...
# https://github.com/coin-or/Gravity/blob/master/cmake/FindIPOPT.cmake
if(DEFINED ENV{IPOPT_HOME})
  set(IPOPT_ROOT_DIR "$ENV{IPOPT_HOME}")
  if (NOT IPOPT_FIND_QUIETLY)
    message("-- Looking for Ipopt in ${IPOPT_ROOT_DIR}")
  endif()
endif()

find_path(
  IPOPT_INCLUDE_DIR
  NAMES IpNLP.hpp
  HINTS ${IPOPT_ROOT_DIR}/include/coin
  HINTS ${IPOPT_ROOT_DIR}/include
  HINTS /usr/include/coin
  HINTS C:/msys64/mingw64/include/coin-or)

if(APPLE)
  find_library(
    IPOPT_LIBRARY libipopt.dylib
    HINTS ${IPOPT_ROOT_DIR}/lib
    HINTS /usr/local/lib)
elseif(UNIX)
  find_library(
    IPOPT_LIBRARY libipopt.so
    HINTS ${IPOPT_ROOT_DIR}/lib
    HINTS /usr/local/lib)
elseif(WIN32)
  find_library(
    IPOPT_LIBRARY
    NAMES ipopt
    HINTS ${IPOPT_ROOT_DIR}/lib
    HINTS C:/msys64/mingw64/lib)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(IPOPT DEFAULT_MSG IPOPT_LIBRARY
                                  IPOPT_INCLUDE_DIR)

if(IPOPT_FOUND)
  if (NOT IPOPT_FIND_QUIETLY)
    message("—- Found IPOPT under ${IPOPT_INCLUDE_DIR}")
  endif()
  set(IPOPT_INCLUDE_DIRS ${IPOPT_INCLUDE_DIR})
  set(IPOPT_LIBRARIES ${IPOPT_LIBRARY})
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(IPOPT_LIBRARIES "${IPOPT_LIBRARIES};m;pthread")
  endif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
endif(IPOPT_FOUND)

mark_as_advanced(IPOPT_LIBRARY IPOPT_INCLUDE_DIR)
