cmake_minimum_required(VERSION 2.8)
project(tams)

if(POLICY CMP0017)
  cmake_policy(SET CMP0017 NEW)
endif()
set(CMAKE_BUILD_TYPE Release)

#find packages
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(WORKSPACE /home/xiao/workspace/pcl)
set(SOFT /home/xiao/workspace/soft)


#cgal package
find_package(CGAL QUIET COMPONENTS Core )
if ( CGAL_FOUND )
  include( ${CGAL_USE_FILE} )
  include( CGAL_CreateSingleSourceCGALProgram )
  include_directories (BEFORE include)
else()
    message(STATUS "This program requires the CGAL library, and will not be compiled.")
endif()

#pcl package
find_package(PCL 1.6)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#opencv
FIND_PACKAGE( OpenCV REQUIRED )
#Eigen
#find_package(Eigen REQUIRED)
#include_directories(${EIGEN_INCLUDE_DIRS})
#SOFT
link_directories(${SOFT})
include_directories(${SOFT}/include)


FILE(GLOB_RECURSE new_list *.h)
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path}/../../include)
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
INCLUDE_DIRECTORIES(${dir_list})


file( GLOB list "*" )
foreach( entry ${list} )
  if( IS_DIRECTORY ${entry} )
    if( EXISTS ${entry}/CMakeLists.txt )
      add_subdirectory( ${entry} )
      link_directories( ${entry}/lib )
      message( STATUS "Configuring  ${entry} xiao" )
    endif()
  endif()
endforeach()

