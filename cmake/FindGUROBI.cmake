if(DEFINED ENV{GUROBI_HOME})
  set(GUROBI_ROOT_DIR "$ENV{GUROBI_HOME}")

  if(GUROBI_ROOT_DIR AND NOT GUROBI_FIND_QUIETLY)
    message("-- Looking for GUROBI and components in ${GUROBI_ROOT_DIR}")
  endif()
endif()

set(GUROBI_HINTS ${GUROBI_ROOT_DIR})
		  
if(APPLE)
  set(GUROBI_API libgurobi_stdc++.a)
  set(GUROBI_NAMES libgurobi95.dylib libgurobi91.dylib libgurobi90.dylib libgurobi80.dylib)
  set(GUROBI_HINTS "${GUROBI_HINTS}"
	/opt/gurobi912/linux64 /opt/gurobi911/linux64 /opt/gurobi901/linux64
        /opt/gurobi811/linux64 /opt/gurobi801/linux64)
elseif(UNIX)
  set(GUROBI_API libgurobi_g++5.2.a)
  set(GUROBI_NAMES libgurobi95.so libgurobi91.so libgurobi90.so libgurobi81.so libgurobi80.so)
  set(GUROBI_HINTS "${GUROBI_HINTS}"
	/opt/gurobi912/linux64 /opt/gurobi911/linux64 /opt/gurobi901/linux64
        /opt/gurobi811/linux64 /opt/gurobi801/linux64)
elseif(WIN32)
  set(GUROBI_API libgurobi_c++.a)
  set(GUROBI_NAMES gurobi95.lib gurobi91.lib gurobi90.lib gurobi81.lib gurobi80.lib)
  set(GUROBI_HINTS "${GUROBI_HINTS}"
	C:/gurobi912/win64 C:/gurobi911/win64 C:/gurob901/win64
        C:/gurobi811/win64 C:/gurobi801/win64)
endif()

set(INC_PATH "")
set(LIB_PATH "")

foreach(hint ${GUROBI_HINTS})
    set(INC_PATH ${INC_PATH} ${hint}/include)
	set(LIB_PATH ${LIB_PATH} ${hint}/lib)
endforeach()

find_path(
	GUROBI_INCLUDE_DIR
	NAMES gurobi_c++.h
	HINTS ${INC_PATH})
find_library(
	GUROBI_LIBRARY
	NAMES  ${GUROBI_NAMES}
	HINTS ${LIB_PATH})
find_library(
    GUROBI_LIBRARY_API
	NAMES ${GUROBI_API}
    HINTS ${LIB_PATH})

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
mark_as_advanced(GUROBI_LIBRARY_API GUROBI_LIBRARY GUROBI_INCLUDE_DIR)
