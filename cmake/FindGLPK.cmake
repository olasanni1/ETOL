if(DEFINED ENV{GLPK_HOME})
  set(GLPK_ROOT_DIR "$ENV{GLPK_HOME}")
  if (NOT GLPK_FIND_QUIETLY)
    message("-- Looking for GLPK in ${GLPK_ROOT_DIR}")
  endif()
endif()

find_path(GLPK_INCLUDE_DIR glpk.h
          HINTS ${GLPK_ROOT_DIR}/include /c/msys64/mingw64/include
                /usr/local/include /usr/include)

find_library(
  GLPK_LIBRARY
  NAMES libglpk.dll.a libglpk.so
  HINTS ${GLPK_ROOT_DIR}/lib /c/msys64/mingw64/lib /usr/local/lib /usr/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GLPK DEFAULT_MSG GLPK_LIBRARY
                                  GLPK_INCLUDE_DIR)

if(GLPK_FOUND)
  if (NOT GLPK_FIND_QUIETLY)
    message("—- Found GLPK under ${GLPK_INCLUDE_DIR}")
  endif()
  set(GLPK_INCLUDE_DIRS ${GLPK_INCLUDE_DIR})
  set(GLPK_LIBRARIES ${GLPK_LIBRARY})
endif(GLPK_FOUND)

mark_as_advanced(GLPK_LIBRARY GLPK_INCLUDE_DIR)
