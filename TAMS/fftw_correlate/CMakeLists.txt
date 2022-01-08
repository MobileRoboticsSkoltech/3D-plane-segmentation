project(fftw_correlate)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(include)


set(srcs
    src/fftw_correlate.cpp
    src/softFFTWCorrelateReal.c
    ${SOFT}/common/csecond.c
    ${SOFT}/common/makeweights.c
    ${SOFT}/lib1/so3_correlate_fftw.c
    ${SOFT}/lib1/soft_fftw.c
    ${SOFT}/lib1/s2_cospmls.c
    ${SOFT}/lib1/s2_legendreTransforms.c
    ${SOFT}/lib1/s2_semi_memo.c
    ${SOFT}/lib1/wrap_soft_fftw_cor2.c)

add_library (fftw_correlate ${srcs})
target_link_libraries(fftw_correlate fftw3)
target_link_libraries(fftw_correlate pcl_visualization pcl_io pcl_surface pcl_features pcl_search pcl_filters)