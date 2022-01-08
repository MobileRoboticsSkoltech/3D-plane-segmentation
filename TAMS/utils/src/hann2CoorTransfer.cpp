#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> output;

  if(argc!=2)
  {
  PCL_INFO("Please give the name of the pcd file");
	return (-1);
  }
  if (pcl::io::loadPCDFile <pcl::PointXYZ> (argv[1], cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd");
    return (-1);
  }
  PCL_INFO ("Loaded %d data points from %s.\n",
            (int)(cloud.width * cloud.height), argv[1]);
  output.points.resize (cloud.points.size ());
  output.height = cloud.height;
  output.width = cloud.width;
  for(int i=0;i<cloud.height;++i)
  {
      for(int j=0;j<cloud.width;++j)
      {
        if ((cloud.points[i*cloud.width+j].x != 0) ||
            (cloud.points[i*cloud.width+j].y != 0) ||
            (cloud.points[i*cloud.width+j].z != 0))
        {
          output.points[i*cloud.width+j].x = -cloud.points[i*cloud.width+j].x;
          output.points[i*cloud.width+j].y = cloud.points[i*cloud.width+j].z;
          output.points[i*cloud.width+j].z = cloud.points[i*cloud.width+j].y - 1.0;
        }
        else
        {
          output.points[i*cloud.width+j].x = 0;
          output.points[i*cloud.width+j].y = 0;
          output.points[i*cloud.width+j].z = 0;
        }
      }
  }
  pcl::io::savePCDFileASCII (argv[1], output);
  return (0);
}
