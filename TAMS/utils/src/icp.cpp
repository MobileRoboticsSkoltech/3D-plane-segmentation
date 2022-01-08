#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>

int
main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_registered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

  struct timeval tpstart,tpend;
  double timeuse;
  std::vector<double> times;
  int start_index = 1, end_index = 20;
  bool downsample = false;
  std::string prefix("/media/work/datasets/BRL/unorganized/scan");
  std::string source_pcd_file, target_pcd_file, source_pose_file, target_pose_file;
  std::string target_transformation_file;
  Eigen::Matrix4f global_transformation = Eigen::Matrix4f::Identity();
  for (int index = start_index; index < end_index; index++)
  {
    char buf[4];
    double tmp;

    sprintf(buf, "%03d", index);
    source_pcd_file = prefix + std::string(buf) + ".pcd";
    source_pose_file = prefix + std::string(buf) + ".pose";
    if (index == start_index)
    {
      target_transformation_file = prefix + std::string(buf) + ".transform";
      std::ofstream target_transformation_out (target_transformation_file.c_str());
      target_transformation_out << global_transformation;
    }
    double source_odometry[3];
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (source_pcd_file, *source_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", source_pcd_file.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
              source_cloud->points.size(), source_cloud->width, source_cloud->height, source_pcd_file.c_str());
    std::ifstream source_pose_in(source_pose_file.c_str());
    source_pose_in >> source_odometry[0] >> source_odometry[1] >> tmp >> tmp >> tmp >> source_odometry[2];
    PCL_INFO ("source odometry (x, y, theta): (%f, %f, %f)!\n", source_odometry[0], source_odometry[1], source_odometry[2]);

    sprintf(buf, "%03d", index + 1);
    target_pcd_file = prefix + std::string(buf) + ".pcd";
    target_pose_file = prefix + std::string(buf) + ".pose";
    target_transformation_file = prefix + std::string(buf) + ".transform";
    double target_odometry[3];
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (target_pcd_file, *target_cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", target_pcd_file.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
              target_cloud->points.size(), target_cloud->width, target_cloud->height, target_pcd_file.c_str());
    std::ifstream target_pose_in (target_pose_file.c_str());
    target_pose_in >> target_odometry[0] >> target_odometry[1] >> tmp >> tmp >> tmp >> target_odometry[2];
    PCL_INFO ("target odometry (x, y, theta): (%f, %f, %f)!\n", target_odometry[0], target_odometry[1], target_odometry[2]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr src (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    if (downsample)
    {
      grid.setLeafSize (0.05, 0.05, 0.05);
      grid.setInputCloud (source_cloud);
      grid.filter (*src);

      grid.setInputCloud (target_cloud);
      grid.filter (*tgt);
    }
    else
    {
      src = source_cloud;
      tgt = target_cloud;
    }

    Eigen::Vector3d pose_src(source_odometry[0], source_odometry[1], source_odometry[2]);
    Eigen::Vector3d pose_tgt(target_odometry[0], target_odometry[1], target_odometry[2]);
    pose_tgt(2) = pose_tgt(2) * M_PI / 180;
    pose_src(2) = pose_src(2) * M_PI / 180;

    Eigen::Matrix4f guess(Eigen::Matrix4f::Identity());
    Eigen::Vector3d translation(pose_src(0) - pose_tgt(0), pose_src(1) - pose_tgt(1), 0);
    Eigen::Vector3d x_base_tgt = Eigen::Vector3d(cos(pose_tgt(2)), sin(pose_tgt(2)), 0);
    Eigen::Vector3d y_base_tgt = Eigen::Vector3d(-sin(pose_tgt(2)), cos(pose_tgt(2)),0);
    Eigen::Vector3d z_base_tgt = Eigen::Vector3d(0,0,1);
    x_base_tgt = x_base_tgt.normalized();
    y_base_tgt = y_base_tgt.normalized();
    Eigen::Vector3d x_base_src = Eigen::Vector3d(cos(pose_src(2)), sin(pose_src(2)), 0);
    Eigen::Vector3d y_base_src = Eigen::Vector3d(-sin(pose_src(2)), cos(pose_src(2)),0);
    Eigen::Vector3d z_base_src = Eigen::Vector3d(0,0,1);
    x_base_src = x_base_src.normalized();
    y_base_src = y_base_src.normalized();

    guess(0,0) = x_base_tgt.dot(x_base_src);
    guess(0,1) = x_base_tgt.dot(y_base_src);
    guess(0,2) = x_base_tgt.dot(z_base_src);
    guess(1,0) = y_base_tgt.dot(x_base_src);
    guess(1,1) = y_base_tgt.dot(y_base_src);
    guess(1,2) = y_base_tgt.dot(z_base_src);
    guess(2,0) = z_base_tgt.dot(x_base_src);
    guess(2,1) = z_base_tgt.dot(y_base_src);
    guess(2,2) = z_base_tgt.dot(z_base_src);
    guess(3,0) = 0.0;
    guess(3,1) = 0.0;
    guess(3,2) = 0.0;
    guess(3,3) = 1.0;
    guess(0,3) = x_base_tgt.dot(translation);
    guess(1,3) = y_base_tgt.dot(translation);
    guess(2,3) = z_base_tgt.dot(translation);

    gettimeofday(&tpstart,NULL);
    // Set the input target
    icp.setInputTarget (tgt);
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.1);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-6);
    // Set the euclidean distance difference epsilon (criterion 3)
    //icp.setEuclideanFitnessEpsilon (0.01);

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity ();
    Eigen::Matrix4f prev = Eigen::Matrix4f::Identity ();
    pcl::PointCloud<pcl::PointXYZ>::Ptr reg_result = src;
//    guess = Eigen::Matrix4f::Identity();
    icp.setMaximumIterations (1);
    for (int i = 0; i < 30; ++i)
    {
      PCL_INFO ("Iteration Nr. %d, Maximum correspondence distance %f.\n", i, icp.getMaxCorrespondenceDistance());
      //save cloud for visualization purpose
      src = reg_result;
      //Estimate
      icp.setInputCloud (src);
      //registration with odometry pose estimation
      if (i == 0)
      {
        icp.align (*reg_result, guess);
      }
      else
      {
        icp.align (*reg_result);
      }
      //accumulate transformation between each Iteration
      Ti = icp.getFinalTransformation () * Ti;
      //if the difference between this transformation and the previous one
      //is smaller than the threshold, refine the process by reducing
      //the maximal correspondence distance
      //if ((icp.getLastIncrementalTransformation () - prev).array ().abs ().sum () < icp.getTransformationEpsilon ())
      //  icp.setMaxCorrespondenceDistance (icp.getMaxCorrespondenceDistance () - 0.01);
      //prev = icp.getLastIncrementalTransformation ();
      if (fabs(icp.getFinalTransformation () .array ().abs ().sum () - 4) < icp.getTransformationEpsilon ())
        icp.setMaxCorrespondenceDistance (icp.getMaxCorrespondenceDistance () - 0.01);
      //std::cout << fabs(icp.getFinalTransformation ().array ().abs ().sum () - 4) << std::endl;
    }
    gettimeofday(&tpend,NULL);
    timeuse=1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec;
    timeuse/=1000000;
    std::cout << "ICP registration time: " << timeuse << std::endl;
    times.push_back(timeuse);

    Ti = Ti.inverse();
    global_transformation = global_transformation * Ti;
    std::ofstream target_transformation_out (target_transformation_file.c_str());
    target_transformation_out << global_transformation;
    pcl::transformPointCloud(*source_cloud, *source_cloud_registered, Ti);
    pcl::visualization::PCLVisualizer viewer ("visualization for translation");
//    viewer.addPointCloud (target_cloud, "source_cloud", 0);
//    viewer.addPointCloud (reg_result, "source_cloud_registered", 0);
//    viewer.addCoordinateSystem (1.0);
//    viewer.setBackgroundColor (1,1,1);
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud_registered");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "source_cloud");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "source_cloud_registered");
//    viewer.spin ();
  }
  for (int i = 0; i < times.size(); i++)
    std::cout << times[i] << " ";
  std::cout << std::endl;
  return 0;
}

