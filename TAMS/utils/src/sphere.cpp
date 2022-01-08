#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
int
main (int argc, char** argv)
{
    std::string pcd_file(argv[1]);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.points.resize (360*180);
    cloud.width = 360 * 180;
    cloud.height = 1;
    for (int i = 0; i < 360; i++)
    {
        for (int j = 0; j < 180; j++)
        {
            cloud.points[i * 180 + j].x = 30 * sin(j * M_PI / 180) * cos(i * M_PI / 180);
            cloud.points[i * 180 + j].y = 30 * sin(j * M_PI / 180) * sin(i * M_PI / 180);
            cloud.points[i * 180 + j].z = 30 * cos(j * M_PI / 180);
        }
    }

    pcl::io::savePCDFile(pcd_file, cloud);

    return (0);
}
