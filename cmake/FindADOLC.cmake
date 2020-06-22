# Modified copy from ...
# https://github.com/joaoleal/CppADCodeGen/blob/master/cmake/FindADOLC.cmake

# * Try to find Adolc Once done this will define ADOLC_FOUND - System has Adolc
#   ADOLC_INCLUDE_DIRS - The Adolc include directories ADOLC_LIBRARY_DIRS - The
#   library directories needed to use Adolc ADOLC_LIBRARIES    - The libraries
#   needed to use Adolc
if(DEFINED ENV{ADOLC_HOME})
  set(ADOLC_DIR "$ENV{ADOLC_HOME}")
  message("-- Looking for ADOLC in ${ADOLC_DIR}")
  if(DEFINED ENV{COLPACK_HOME})
    set(COLPACK_DIR "$ENV{COLPACK_HOME}")
    if (NOT ADOLC_FIND_QUIETLY)
      message("-- Looking for COLPACK in ${COLPACK_DIR}")
    endif()
  else()
    if (NOT ADOLC_FIND_QUIETLY)
      message("-- Looking for COLPACK in ${ADOLC_DIR}")
    endif()
  endif()
endif()

find_path(
  ADOLC_INCLUDE_DIR
  NAMES adouble.h
  HINTS ${ADOLC_DIR}/include ${ADOLC_DIR}/include/adolc
        C:/msys64/mingw64/include/adolc /usr/include/adolc)

find_library(ADOLC_LIBRARY adolc HINTS ${ADOLC_DIR}/lib ${ADOLC_DIR}/lib64
                                       /usr/lib C:/msys64/mingw64/lib)
if(COLPACK_DIR)
  find_library(COLPACK_LIBRARY ColPack HINTS ${COLPACK_DIR}/lib /usr/lib
                                             C:/msys64/mingw64/lib)
else()
  find_library(COLPACK_LIBRARY ColPack HINTS ${ADOLC_DIR}/lib /usr/lib
                                             C:/msys64/mingw64/lib)
endif()

find_package(OpenMP)
include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LIBIPOPT_FOUND to TRUE if
# all listed variables are TRUE
find_package_handle_standard_args(ADOLC DEFAULT_MSG ADOLC_LIBRARY
                                  ADOLC_INCLUDE_DIR COLPACK_LIBRARY)

if(ADOLC_FOUND)
  if (NOT ADOLC_FIND_QUIETLY)
    message("—- Found ADOLC under ${ADOLC_INCLUDE_DIR}")
  endif()
  set(ADOLC_INCLUDE_DIRS ${ADOLC_INCLUDE_DIR})
  set(ADOLC_LIBRARIES ${ADOLC_LIBRARY} ${COLPACK_LIBRARY}
  ${OpenMP_C_LIBRARIES})
endif()
mark_as_advanced(ADOLC_INCLUDE_DIR ADOLC_LIBRARY)
