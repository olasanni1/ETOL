cmake_minimum_required(VERSION 2.8.9)

project(TrajectoryOptimizer)

find_package(LibXml2 REQUIRED)

find_package(Boost 1.58.0 COMPONENTS thread system filesystem iostreams
                          REQUIRED)

add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}.cpp)

target_compile_definitions(${PROJECT_NAME} PUBLIC CGAL_HEADER_ONLY)

target_link_libraries(${PROJECT_NAME}
  PRIVATE gmp  ${LIBXML2_LIBRARIES} ${Iconv_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY}
  ${Boost_FILESYSTEM_LIBRARY} ${Boost_IOSTREAMS_LIBRARY})

target_include_directories(${PROJECT_NAME}
  PRIVATE ${LIBXML2_INCLUDE_DIR} ${Iconv_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
