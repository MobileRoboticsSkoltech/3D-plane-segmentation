# This module setups the compiler for using Eigen v3 library.
# It assumes that find_package(Eigen3) was already called.


include_directories( SYSTEM ${EIGEN3_INCLUDE_DIR} )

add_definitions(-DCGAL_EIGEN3_ENABLED)

set (EIGEN3_SETUP TRUE)
