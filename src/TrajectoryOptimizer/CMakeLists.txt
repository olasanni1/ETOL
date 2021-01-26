cmake_minimum_required(VERSION 3.5)

project(TrajectoryOptimizer)

find_package(LibXml2 REQUIRED)
find_package(Boost COMPONENTS thread system filesystem iostreams)
find_package(CGAL)
find_package(re2)

add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME}
  PUBLIC gmp  ${LIBXML2_LIBRARIES} ${Iconv_LIBRARIES}
  $<IF:$<BOOL:${CGAL_FOUND}>,CGAL::CGAL,>
  $<IF:$<BOOL:${re2_FOUND}>,re2::re2,re2>
  $<IF:$<BOOL:${Boost_THREAD_FOUND}>,Boost::thread,>
  $<IF:$<BOOL:${Boost_SYSTEM_FOUND}>,Boost::system,>
  $<IF:$<BOOL:${Boost_FILESYSTEM_FOUND}>,Boost::filesystem,>
  $<IF:$<BOOL:${Boost_IOSTREAMS_FOUND}>,Boost::iostreams,>)
 
if(NOT CGAL_FOUND)
  target_compile_definitions(${PROJECT_NAME} PRIVATE CGAL_HEADER_ONLY)
endif()

target_include_directories(${PROJECT_NAME}
  PUBLIC ../../include
  PRIVATE ${LIBXML2_INCLUDE_DIR} ${Iconv_INCLUDE_DIR})
