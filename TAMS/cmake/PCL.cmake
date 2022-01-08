###############################################################################
# Find PCL
#
find_package(PCL 1.6 REQUIRED)
if (PCL_FOUND)
	include_directories(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
else
	message (STATUS "This program requires the PCL library which is not found, so it will not be compiled.")




