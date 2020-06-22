if(DEFINED ENV{PSOPT_HOME})
  set(PSOPT_ROOT_DIR "$ENV{PSOPT_HOME}")
  set(PSOPT_DIR ${PSOPT_ROOT_DIR}/PSOPT)
  set(DMATRIX_DIR ${PSOPT_ROOT_DIR}/dmatrix)
  set(LUSOL_DIR ${PSOPT_ROOT_DIR}/lusol)

  if (NOT PSOPT_FIND_QUIETLY)
    message("-- Looking for PSOPT and components in ${PSOPT_ROOT_DIR}")
    message("-- Looking for PSOPT ${PSOPT_DIR}")
    message("-- Looking for DMatrix in ${DMATRIX_DIR}")
    message("-- Looking for lusol in ${LUSOL_DIR}")
  endif()
endif()

# PSOPT
find_path(
  PSOPT_INCLUDE_DIR
  NAMES psopt.h
  HINTS ${PSOPT_DIR}/src)

find_library(PSOPT_LIBRARY libpsopt.a HINTS ${PSOPT_DIR}/lib)

# DMATRIX
find_path(
  DMATRIX_INCLUDE_DIR
  NAMES dmatrixv.h
  HINTS ${DMATRIX_DIR}/include)

find_library(DMATRIX_LIBRARY libdmatrix.a HINTS ${DMATRIX_DIR}/lib)

# lusol
find_library(LUSOL_LIBRARY liblusol.a HINTS ${LUSOL_DIR})

if (NOT PSOPT_FIND_QUIETLY)
  find_package(IPOPT)
  find_package(CXSparse)
  find_package(ADOLC)
  find_package(LAPACK)
else()
  find_package(IPOPT QUIET)
  find_package(CXSparse QUIET)
  find_package(ADOLC QUIET)
  find_package(LAPACK QUIET)
endif()

if(CXSparse_FOUND
   AND IPOPT_FOUND
   AND ADOLC_FOUND
   AND LAPACK_FOUND)
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(
    PSOPT
    DEFAULT_MSG
    PSOPT_LIBRARY
    PSOPT_INCLUDE_DIR
    DMATRIX_LIBRARY
    DMATRIX_INCLUDE_DIR
    LUSOL_LIBRARY)
else()
  set(PSOPT_FOUND OFF)
endif()
if(PSOPT_FOUND)
  if (NOT PSOPT_FIND_QUIETLY)
    message("—- Found PSOPT under ${PSOPT_INCLUDE_DIR}")
  endif()
  set(PSOPT_INCLUDE_DIRS
      ${PSOPT_INCLUDE_DIR} ${DMATRIX_INCLUDE_DIR} ${IPOPT_INCLUDE_DIRS}
      ${CXSPARSE_INCLUDE_DIRS} ${ADOLC_INCLUDE_DIRS})
  set(PSOPT_LIBRARIES
      ${PSOPT_LIBRARY}
      ${DMATRIX_LIBRARY}
      ${LUSOL_LIBRARY}
      ${IPOPT_LIBRARIES}
      ${BLAS_LIBRARIES}
      ${LAPACK_LIBRARIES}
      ${CXSPARSE_LIBRARIES}
      ${ADOLC_LIBRARIES}
      "-ldl"
      "-lm")
  set(PSOPT_DEFINITIONS LAPACK USE_IPOPT SPARSE_MATRIX)
  if(UNIX)
    list(APPEND PSOPT_DEFINITIONS UNIX)
  endif()
endif(PSOPT_FOUND)

mark_as_advanced(PSOPT_LIBRARY PSOPT_INCLUDE_DIR DMATRIX_LIBRARY
                 DMATRIX_INCLUDE_DIR LUSOL_LIBRARY)
