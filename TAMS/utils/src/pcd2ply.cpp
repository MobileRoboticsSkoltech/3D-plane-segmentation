#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <string>
int
main(int argc, char **argv)
{
  pcl::PCDReader pcd_reader;
  pcl::PLYWriter ply_writer;
  std::string ply_file;
  std::string pcd_file;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (argc != 3)
  {
    std::cout <<
     "Syntax is: pcd2ply <file_in.pcd> <file_out.ply>!" << std::endl;
    return -1;
  }
  pcd_file = argv[1];
  ply_file = argv[2];

  pcd_reader.read<pcl::PointXYZ> (pcd_file, cloud);
  ply_writer.write(ply_file, cloud);

  return 0;
}
