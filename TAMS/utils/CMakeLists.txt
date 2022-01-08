project(utils)
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(include)

add_executable(setupSimu src/setupSimu.cpp)
target_link_libraries(setupSimu ${PCL_LIBRARIES})

add_executable(voxel_downsampling src/voxel_downsampling.cpp)
target_link_libraries(voxel_downsampling ${PCL_LIBRARIES} pcl_filters)

add_executable(sphere src/sphere.cpp)
target_link_libraries(sphere ${PCL_LIBRARIES})

add_executable(max_range src/max_range.cpp)
target_link_libraries(max_range ${PCL_LIBRARIES})

add_executable(pcl_icpn src/pcl_icpn.cpp)
target_link_libraries(pcl_icpn pcl_features pcl_filters ${PCL_LIBRARIES})

add_executable(pcl_icp src/pcl_icp.cpp)
target_link_libraries(pcl_icp pcl_features pcl_filters ${PCL_LIBRARIES})

add_executable(jacobs_to_organized src/jacobs_to_organized.cpp)
target_link_libraries(jacobs_to_organized ${PCL_LIBRARIES})

add_executable(origin2nan src/origin2nan.cpp)
target_link_libraries(origin2nan ${PCL_LIBRARIES})

add_executable(pcd2depth src/pcd2depth.cpp)
target_link_libraries(pcd2depth ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(mm2m src/mm2m.cpp)
target_link_libraries(mm2m ${PCL_LIBRARIES})

add_executable(m2mm src/m2mm.cpp)
target_link_libraries(m2mm ${PCL_LIBRARIES})

add_executable(show src/show.cpp)
target_link_libraries(show ${PCL_LIBRARIES})
target_link_libraries(show common)

add_executable(show_pair_with_pose src/show_pair_with_pose.cpp)
target_link_libraries(show_pair_with_pose ${PCL_LIBRARIES})

add_executable(remove_nans src/remove_nans.cpp)
target_link_libraries(remove_nans ${PCL_LIBRARIES})

add_executable(boundingbox src/boundingbox.cpp)
target_link_libraries(boundingbox ${PCL_LIBRARIES})

add_executable(icp src/icp.cpp)
target_link_libraries(icp pcl_features pcl_filters pcl_registration ${PCL_LIBRARIES})

add_executable(odometry_registration src/odometry_registration.cpp)
target_link_libraries(odometry_registration ${PCL_LIBRARIES})
