#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include "pcl/search/pcl_search.h"
#include <string>
#include <cmath>

void pitch();
void yaw();
void heatmapRGB (double gray, uint8_t &r, uint8_t &g, uint8_t &b);

int main(int argc, char **argv)
{
  yaw();
  return 0;
}

void yaw()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr sphere (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_sphere(new pcl::PointCloud<pcl::PointXYZRGB>);
  sphere->resize(360 * 136);
  sphere->height = 360;
  sphere->width = 136;
  color_sphere->resize(360 * 136);
  color_sphere->height = 360;
  color_sphere->width = 136;

  std::vector<double > cphi;
  cphi.resize(136);
  std::vector<double > sphi;
  sphi.resize(136);

  for (size_t i = 0; i < 136; i++)
  {
    cphi[i] = cos((-135.0f + static_cast<double >(i)) * M_PI / 180.0f);
    sphi[i] = sin((-135.0f + static_cast<double >(i)) * M_PI / 180.0f);

  }

  for (size_t i = 0; i < 360; i++)
  {
    double ctheta = cos(static_cast<double >(i) * M_PI / 180.0f);
    double stheta = sin(static_cast<double >(i) * M_PI / 180.0f);
    std::cout << ctheta << ", " << stheta << std::endl;
    for (size_t j = 0; j < 136; j++)
    {
      sphere->points[i * sphere->width + j].x = sphi[j] * stheta;
      sphere->points[i * sphere->width + j].y = sphi[j] * ctheta;
      sphere->points[i * sphere->width + j].z = cphi[j];
     // std::cout << sphere->points[i * sphere->width + j].x << ", " << sphere->points[i * sphere->width + j].y << ", " << sphere->points[i * sphere->width + j].z << std::endl;
    }
  }

  pcl::search::Search<pcl::PointXYZ>::Ptr tree;
  tree.reset(new pcl::search::KdTree<pcl::PointXYZ> (false));
  tree->setInputCloud (sphere);

  std::vector<double > dis;
  dis.resize(sphere->size());
  std::vector<int> nn_indices;
  nn_indices.resize(2);
  std::vector<float> nn_dis;
  nn_dis.resize(2);
  for (size_t i = 0; i < sphere->size(); i++)
  {
    tree->nearestKSearch(i, 2, nn_indices, nn_dis);
    dis[i] = nn_dis[1];
  }
  double max_dis = 0;
  for (size_t i = 0; i < sphere->size(); i++)
  {
    if (dis[i] > max_dis)
      max_dis = dis[i];
  }

  for (size_t i = 0; i < sphere->size(); i++)
  {
    dis[i] = dis[i] / max_dis;
  }

  uint8_t r = 0, g = 0, b = 0;
  uint32_t rgb = 0;
  for (size_t i = 0; i < sphere->size(); i++)
  {
    color_sphere->points[i].x = sphere->points[i].x;
    color_sphere->points[i].y = sphere->points[i].y;
    color_sphere->points[i].z = sphere->points[i].z;
    heatmapRGB(dis[i], r, g, b);
    rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    color_sphere->points[i].rgb = *reinterpret_cast<double *> (&rgb);
  }

  pcl::visualization::PCLVisualizer viewer ("EGI and normals distribution");
  viewer.setBackgroundColor (1.0, 1.0, 1.0);
  viewer.addPointCloud (color_sphere, "pitch", 0);
  viewer.addArrow (pcl::PointXYZ(0.0, 0.0, 1.2), pcl::PointXYZ(0.0, 0.0, 0.0), 0.0, 0.0, 0.0, false);
  viewer.addArrow (pcl::PointXYZ(1.0, 0.0, 0.0), pcl::PointXYZ(0.0, 0.0, 0.0), 0.0, 0.0, 1.0, false, "front");
  viewer.addCube	(	-1.0f, 1.0f, -1.0f, 1.0f, -0.001, 0.001, 0.0f, 0.0f, 0.0f);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "pitch");
  viewer.spin ();
}


void pitch()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr sphere (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_sphere(new pcl::PointCloud<pcl::PointXYZRGB>);

  sphere->resize(121 * 271);
  sphere->height = 121;
  sphere->width = 271;
  sphere->points.resize(121 * 271);
  color_sphere->resize(121 * 271);
  color_sphere->height = 121;
  color_sphere->width = 271;
  color_sphere->points.resize(121 * 271);

  std::vector<double > cphi;
  cphi.resize(271);
  std::vector<double > sphi;
  sphi.resize(271);
  for (size_t i = 0; i < 271; i++)
  {
    cphi[i] = cos((-135.0f + static_cast<double >(i)) * M_PI / 180.0f);
    sphi[i] = sin((-135.0f + static_cast<double >(i)) * M_PI / 180.0f);
   // std::cout << cphi[i] << ", " << sphi[i] << std::endl;
  }
  double ctheta, stheta;
  for (size_t i = 0; i < 31; i++)
  {
    ctheta = cos( (60.0f + static_cast<double >(i)) * M_PI / 180.0f);
    stheta = sin( (60.0f + static_cast<double >(i)) * M_PI / 180.0f);

    for (size_t j = 0; j < 45; j++)
    {
      sphere->points[i * sphere->width + j].x = cphi[j] * stheta;
      sphere->points[i * sphere->width + j].y = sphi[j];
      sphere->points[i * sphere->width + j].z = - cphi[j] * ctheta;
      //std::cout << sphere->points[i * sphere->width + j].x << ", "  << sphere->points[i * sphere->width + j].y << ", " << sphere->points[i * sphere->width + j].z << std::endl;
    }

    for (size_t j = 226; j < 271; j++)
    {
      sphere->points[i * sphere->width + j].x = cphi[j] * stheta;
      sphere->points[i * sphere->width + j].y = sphi[j];
      sphere->points[i * sphere->width + j].z = - cphi[j] * ctheta;
      //std::cout << sphere->points[i * sphere->width + j].x << ", "  << sphere->points[i * sphere->width + j].y << ", " << sphere->points[i * sphere->width + j].z << std::endl;
    }

    ctheta = cos((120.0f - static_cast<double >(i)) * M_PI / 180.0f);
    stheta = sin((120.0f - static_cast<double >(i)) * M_PI / 180.0f);

    for (size_t j = 45; j < 226; j++)
    {
      sphere->points[i * sphere->width + j].x = cphi[j] * stheta;
      sphere->points[i * sphere->width + j].y = sphi[j];
      sphere->points[i * sphere->width + j].z = cphi[j] * ctheta;
      //std::cout << sphere->points[i * sphere->width + j].x << ", "  << sphere->points[i * sphere->width + j].y << ", " << sphere->points[i * sphere->width + j].z << std::endl;
    }

  }


  for (size_t i = 31; i < 121; i++)
  {
    double ctheta = cos( (90.0f + static_cast<double >(i - 30)) * M_PI / 180.0f);
    double stheta = sin( (90.0f + static_cast<double >(i - 30)) * M_PI / 180.0f);

    for (size_t j = 0; j < 45; j++)
    {
      sphere->points[i * sphere->width + j].x = cphi[j] * stheta;
      sphere->points[i * sphere->width + j].y = sphi[j];
      sphere->points[i * sphere->width + j].z = - cphi[j] * ctheta;
    }

    for (size_t j = 226; j < 271; j++)
    {
      sphere->points[i * sphere->width + j].x = cphi[j] * stheta;
      sphere->points[i * sphere->width + j].y = sphi[j];
      sphere->points[i * sphere->width + j].z = - cphi[j] * ctheta;
    }

    ctheta = cos((90.0f - static_cast<double >(i - 30)) * M_PI / 180.0f);
    stheta = sin((90.0f - static_cast<double >(i - 30)) * M_PI / 180.0f);

    for (size_t j = 45; j < 226; j++)
    {
      sphere->points[i * sphere->width + j].x = cphi[j] * stheta;
      sphere->points[i * sphere->width + j].y = sphi[j];
      sphere->points[i * sphere->width + j].z = cphi[j] * ctheta;
    }
  }

  pcl::search::Search<pcl::PointXYZ>::Ptr tree;
  tree.reset(new pcl::search::KdTree<pcl::PointXYZ> (false));
  tree->setInputCloud (sphere);

  std::vector<double > dis;
  dis.resize(sphere->size());
  std::vector<int> nn_indices;
  nn_indices.resize(2);
  std::vector<float> nn_dis;
  nn_dis.resize(2);
  for (size_t i = 0; i < sphere->size(); i++)
  {
    tree->nearestKSearch(i, 2, nn_indices, nn_dis);
    dis[i] = nn_dis[1];
  }
  double max_dis = 0;
  for (size_t i = 0; i < sphere->size(); i++)
  {
    if (dis[i] > max_dis)
      max_dis = dis[i];
  }

  for (size_t i = 0; i < sphere->size(); i++)
  {
    dis[i] = dis[i] / max_dis;
  }

  uint8_t r = 0, g = 0, b = 0;
  uint32_t rgb = 0;
  for (size_t i = 0; i < sphere->size(); i++)
  {
    color_sphere->points[i].x = sphere->points[i].x;
    color_sphere->points[i].y = sphere->points[i].y;
    color_sphere->points[i].z = sphere->points[i].z;
    heatmapRGB(dis[i], r, g, b);
    rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    color_sphere->points[i].rgb = *reinterpret_cast<double *> (&rgb);
  }

  pcl::visualization::PCLVisualizer viewer ("Simulated field of view");
  viewer.setBackgroundColor (1.0, 1.0, 1.0);
  viewer.addPointCloud (color_sphere, "pitch", 0);
  viewer.addArrow (pcl::PointXYZ(0.0, 1.2, 0.0), pcl::PointXYZ(0.0, -1.2, 0.0), 0.0, 0.0, 0.0, false);
  viewer.addArrow (pcl::PointXYZ(1.0, 0.0, 0.0), pcl::PointXYZ(0.0, 0.0, 0.0), 0.0, 0.0, 1.0, false, "front");
  viewer.addCube	(	-1.0f, 1.0f, -1.0f, 1.0f, -0.001, 0.001, 0.0f, 0.0f, 0.0f);
  //viewer.addCoordinateSystem(0.5);
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pitch");
  viewer.spin ();
}


void
heatmapRGB (double gray, uint8_t &r, uint8_t &g, uint8_t &b)
{
  if (gray >= 0.0 && gray <= 0.125)
  {
    r = 0;
    g = 0;
    b = 127 + floor (gray * 128 / 0.125);
  }
  else if (gray > 0.125 && gray <= 0.375)
  {
    r = 0;
    g = floor ((gray - 0.125) * 255 / 0.25);
    b = 255;
  }
  else if (gray > 0.375 && gray <= 0.625)
  {
    r = floor ((gray - 0.375) * 255 / 0.25);
    g = 255;
    b = 255 - floor ((gray - 0.375) * 255 / 0.25);
  }
  else if (gray > 0.625 && gray <= 0.875)
  {
    r = 255;
    g = 255 - floor ((gray - 0.625) * 255 / 0.25);
    b = 0;
  }
  else if (gray > 0.875 && gray <= 1.0)
  {
    r = 255 - floor ((gray - 0.875) * 128 / 0.125);
    g = 0;
    b = 0;
  }
}


