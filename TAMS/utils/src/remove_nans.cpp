#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <math.h>
#include <string>

int
main(int argc, char **argv)
{
  if (argc != 5)
  {
    std::cerr << "usage: remove_nans <direction> <prefix> <start> <end>.  \n";
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output (new pcl::PointCloud<pcl::PointXYZ>);


  std::string prefix = std::string(argv[1]) + std::string(argv[2]); //("/media/work/datasets/bremen_city/pcd/scan");
  int start_index = atoi(argv[3]);
  int end_index = atoi(argv[4]);

  std::string pcd_file;

  pcl::PCDReader reader;
  pcl::PCDWriter writer;

  for (int index = start_index; index <= end_index; index++)
  {
    char buf[5];
    sprintf(buf, "%03d", index);
    pcd_file = prefix + std::string(buf) + ".pcd";

    if (reader.read(pcd_file, *cloud) == -1)
    {
      PCL_ERROR ("Couldn't read file %s!\n", pcd_file.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points: (width %d, height: %d) from %s.\n",
              cloud->points.size(), cloud->width, cloud->height, pcd_file.c_str());

    pcl::copyPointCloud(*cloud, *output);

    int cnt = 0;
    for (size_t i = 0; i < output->size (); i++)
    {
      if (output->points[i].x * output->points[i].x  + output->points[i].y * output->points[i].y + output->points[i].z + output->points[i].z > 900)
      output->points[i].x = output->points[i].y = output->points[i].z = NAN;
      cnt ++;
    }
    std::cout << cnt << std::endl;
    output->is_dense = false;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud (*output, *output, indices);
    std::cout << "width :" << output->width << "; height: " << output->height << std::endl;
    writer.writeASCII (pcd_file, *output);
  }

  return 0;
}








