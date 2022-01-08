/*
 * Software License Agreement (BSD License)
 *
 *  Technical Aspects of Multimodal Systems (TAMS) - http://tams-www.informatik.uni-hamburg.de/
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of TAMS, nor the names of its contributors may
 *     be used to endorse or promote products derived from this software
 *     without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author : Junhao Xiao
 * Email  : junhao.xiao@ieee.org, xiao@informatik.uni-hamburg.de
 */
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
int
main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr sphere (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointXYZ> points;
  if (argc != 4)
  {
    std::cout << "usage: egi <dir> <start> <end>. \n";
    return -1;
  }
  std::string dir(argv[1]);
  int start = atoi(argv[2]);
  int end = atoi(argv[3]);
  int bandwidth = 180;
  sphere->resize(4 * bandwidth * bandwidth);
  doubletmp_theta, tmp_phi;
  for (size_t i = 0; i < 2 * bandwidth; i++)
  {
    tmp_theta = (2 * i + 1) * M_PI / 4 / bandwidth;
    for (size_t j = 0; j < 2 * bandwidth; j++)
    {
      tmp_phi = M_PI * j / bandwidth;
      sphere->points[i * 2 * bandwidth + j].x = 30.0 * cos (tmp_phi) * sin (tmp_theta);
      sphere->points[i * 2 * bandwidth + j].y = 30.0 * sin (tmp_phi) * sin (tmp_theta);
      sphere->points[i * 2 * bandwidth + j].z = 30.0 * cos (tmp_theta);
    }
  }
  for (int scan_index = start; scan_index < end; scan_index++)
  {
    char buf[4];
    sprintf (buf, "%03d", scan_index);
    std::string pcd_file = dir + "scan" + std::string (buf) + ".pcd";
    std::string output_file = std::string(buf) + ".pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", pcd_file.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
              cloud->points.size(), cloud->width, cloud->height, pcd_file.c_str());
    points.resize(cloud->size());
    doublethreshold = 29 * 29;
    size_t count = 0;
    size_t outlier = 0;
    for (size_t i = 0; i < cloud->size(); i++)
    {
      if (cloud->points[i].z < 0 &&
          (cloud->points[i].x * cloud->points[i].x + cloud->points[i].y * cloud->points[i].y) < 0.64)
        continue;
      if (cloud->points[i].x * cloud->points[i].x +
          cloud->points[i].y * cloud->points[i].y +
          cloud->points[i].z * cloud->points[i].z > threshold)
      {
        outlier ++;
        continue;
      }
      points[count] = cloud->points[i];
      count ++;
    }
    PCL_INFO ("%d long range outliers removed.\n", outlier);

    filtered_cloud->resize(count);
    filtered_cloud->points.clear ();
    filtered_cloud->points.insert (filtered_cloud->points.begin(), points.begin(), points.begin() + count);
    filtered_cloud->height = 1;
    filtered_cloud->width = count;
    std::cerr << "Cloud after long range outlier removing: " << std::endl;
    std::cerr << *filtered_cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (filtered_cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*output);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *output << std::endl;


    pcl::visualization::PCLVisualizer viewer ("cloud and range sphere");
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    viewer.addPointCloud(output, "cloud");
    viewer.setCameraPosition (0.0, 0.0, 50.0, 0.0, 0.0, 0.0);
    //viewer.addPointCloud(sphere, "sphere");
    //viewer.addCoordinateSystem (1.0);
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sphere");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud");
    //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "sphere");
    viewer.spin ();
  }
  return 0;
}
