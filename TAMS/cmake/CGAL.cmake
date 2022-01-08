###############################################################################
# Find CGAL
#
find_package(CGAL QUIET COMPONENTS Core )
if ( CGAL_FOUND )
  include( ${CGAL_USE_FILE} )
  include( CGAL_CreateSingleSourceCGALProgram )
  include_directories (BEFORE include)
else()
    message(STATUS "This program requires the CGAL library, and will not be compiled.")
endif()




