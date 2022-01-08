project(apps)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(include)

add_executable(soft ${SOFT}/common/csecond.c  ${SOFT}/common/makeweights.c ${SOFT}/lib1/so3_correlate_fftw.c
                    ${SOFT}/lib1/soft_fftw.c  ${SOFT}/lib1/s2_cospmls.c ${SOFT}/lib1/s2_legendreTransforms.c
                    ${SOFT}/lib1/s2_semi_memo.c ${SOFT}/lib1/wrap_soft_fftw_cor2.c 
                    src/application_options_manager.cpp src/soft.cpp)
target_link_libraries(soft fftw3 soft1)
target_link_libraries(soft boost_program_options)
target_link_libraries(soft ${PCL_LIBRARIES} pcl_search pcl_filters pcl_features)
target_link_libraries(soft fftw_correlate)

add_executable(planar_segmentation src/application_options_manager.cpp src/planar_segmentation.cpp)
target_link_libraries(planar_segmentation ${PCL_LIBRARIES})
target_link_libraries(planar_segmentation boost_program_options boost_filesystem)

add_executable(octree_planar_segmentation src/application_options_manager.cpp src/octree_planar_segmentation.cpp)
target_link_libraries(octree_planar_segmentation pcl_filters pcl_visualization ${PCL_LIBRARIES})
target_link_libraries(octree_planar_segmentation boost_program_options boost_filesystem)
target_link_libraries(octree_planar_segmentation CGAL CGAL_Core)
target_link_libraries(octree_planar_segmentation common octreeRG abstract_planar_segment segments_area)

add_executable(map_segmentation src/application_options_manager.cpp src/map_segmentation.cpp)
target_link_libraries(map_segmentation pcl_filters pcl_visualization ${PCL_LIBRARIES})
target_link_libraries(map_segmentation boost_program_options boost_filesystem)
target_link_libraries(map_segmentation CGAL CGAL_Core)
target_link_libraries(map_segmentation common octreeRG abstract_planar_segment segments_area)

#add_executable(area_beachmarking src/application_options_manager.cpp src/area_benchmark.cpp)
#target_link_libraries(area_beachmarking pcl_filters pcl_io pcl_visualization ${PCL_LIBRARIES})
#target_link_libraries(area_beachmarking boost_program_options boost_filesystem)
#target_link_libraries(area_beachmarking CGAL CGAL_Core)
#target_link_libraries(area_beachmarking common area_benchmark abstract_planar_segment segments_area)

add_executable(segments_descriptor src/segments_descriptor.cpp src/application_options_manager.cpp)
target_link_libraries(segments_descriptor ${PCL_LIBRARIES})
target_link_libraries(segments_descriptor boost_program_options boost_filesystem)

add_executable(loop_detection src/loop_detection.cpp src/application_options_manager.cpp)
target_link_libraries(loop_detection ${PCL_LIBRARIES})
target_link_libraries(loop_detection loop_detector octreeRG abstract_planar_segment)
target_link_libraries(loop_detection boost_program_options)

add_executable(mapper src/registration.cpp src/application_options_manager.cpp)
target_link_libraries(mapper ${PCL_LIBRARIES})
target_link_libraries(mapper boost_program_options boost_filesystem)
target_link_libraries(mapper common octreeRG abstract_planar_segment segments_area registration)

add_executable(map_merger src/map_merging.cpp src/application_options_manager.cpp)
target_link_libraries(map_merger ${PCL_LIBRARIES})
target_link_libraries(map_merger boost_program_options boost_filesystem)
target_link_libraries(map_merger octreeRG abstract_planar_segment segments_area registration)


add_dependencies(octree_planar_segmentation abstract_planar_segment)
