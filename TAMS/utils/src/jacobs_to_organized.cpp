#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>

int
main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::visualization::PCLVisualizer *p;
  p = new pcl::visualization::PCLVisualizer (argc, argv, "show registered clouds");
  p->setBackgroundColor (1,1,1);
  int start_index = 7, end_index = 32;

  std::string prefix("/media/work/datasets/crashedCarPark/jacobs/alrf");
  std::string pcd_file;

  pcl::PCDReader reader;
  pcl::PCDWriter writer;

  for (int index = start_index; index <= end_index; index++)
  {
    char buf[5];
    sprintf(buf, "%04d", index);
    pcd_file = prefix + std::string(buf) + ".pcd";

    if (reader.read(pcd_file, *cloud) == -1)
    {
      PCL_ERROR ("Couldn't read file %s!\n", pcd_file.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
              cloud->points.size(), cloud->width, cloud->height, pcd_file.c_str());


    output->resize(cloud->size());
    output->points.resize(cloud->size());

    int height = cloud->height;
    int width = cloud->width;
    for (size_t i = 0; i < cloud->height; i++)
    {
      for (size_t j = 0; j < 90; j++)
        output->points[i * width + j] = cloud->points[i * width + j];
      for (size_t j = 90; j <= 450; j++)
        output->points[i * width + j] = cloud->points[(height - i - 1) * width + j];
      for (size_t j = 451; j < width; j++)
        output->points[i * width + j] = cloud->points[i * width + j];
    }

    output->height = height;
    output->width = width;

    for (size_t i = 0; i < output->size(); i++)
    {
      if (output->points[i].x == 0 && output->points[i].y == 0 && output->points[i].z == 0)
      {
        output->points[i].x = output->points[i].y = output->points[i].z = NAN;
      }
    }
    output->is_dense = false;

    char tmp[4];
    sprintf(tmp, "%03d", index);
    std::string str = "scan" + std::string(tmp) + ".pcd";
    writer.writeASCII(str, *output);

    p->addPointCloud (output, pcd_file, 0);
    p->addPointCloud(cloud, "cloud", 0);

    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cloud");
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, pcd_file);
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, pcd_file);
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, "cloud");
    p->spin();
    p->removeAllPointClouds();

  }

  return 0;
}



