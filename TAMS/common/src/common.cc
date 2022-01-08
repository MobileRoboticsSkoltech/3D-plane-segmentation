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
 *
 */
#include "common/common.h"

namespace tams
{
  void
  getColors(std::vector<RGB> &colors)
  {
    colors.clear();
    colors.push_back(RGB(0x99, 0xCC, 0x32));
    colors.push_back(RGB(0xFF, 0xFF, 0x00));
    colors.push_back(RGB(0xFF, 0xFF, 0xFF));
    colors.push_back(RGB(0xD8, 0xD8, 0xBF));
    colors.push_back(RGB(0xCC, 0x32, 0x99));
    colors.push_back(RGB(0x4F, 0x2F, 0x4F));
    colors.push_back(RGB(0xCD, 0xCD, 0xCD));
    colors.push_back(RGB(0x5C, 0x40, 0x33));
    colors.push_back(RGB(0xAD, 0xEA, 0xEA));
    colors.push_back(RGB(0xD8, 0xBF, 0xD8));
    colors.push_back(RGB(0xDB, 0x93, 0x70));
    colors.push_back(RGB(0x38, 0xB0, 0xDE));
    colors.push_back(RGB(0x23, 0x6B, 0x8E));
    colors.push_back(RGB(0x00, 0xFF, 0x7F));
    colors.push_back(RGB(0xFF, 0x1C, 0xAE));
    colors.push_back(RGB(0x00, 0x7F, 0xFF));
    colors.push_back(RGB(0x32, 0x99, 0xCC));
    colors.push_back(RGB(0xE6, 0xE8, 0xFA));
    colors.push_back(RGB(0x8E, 0x6B, 0x23));
    colors.push_back(RGB(0x6B, 0x42, 0x26));
    colors.push_back(RGB(0x23, 0x8E, 0x68));
    colors.push_back(RGB(0x8C, 0x17, 0x17));
    colors.push_back(RGB(0x6F, 0x42, 0x42));
    colors.push_back(RGB(0x59, 0x59, 0xAB));
    colors.push_back(RGB(0xFF, 0x00, 0x00));
    colors.push_back(RGB(0xD9, 0xD9, 0xF3));
    colors.push_back(RGB(0xEA, 0xAD, 0xEA));
    colors.push_back(RGB(0xBC, 0x8F, 0x8F));
    colors.push_back(RGB(0x8F, 0xBC, 0x8F));
    colors.push_back(RGB(0xDB, 0x70, 0xDB));
    colors.push_back(RGB(0xFF, 0x24, 0x00));
    colors.push_back(RGB(0xFF, 0x7F, 0x00));
    colors.push_back(RGB(0xCF, 0xB5, 0x3B));
    colors.push_back(RGB(0xEB, 0xC7, 0x9E));
    colors.push_back(RGB(0x00, 0x00, 0x9C));
    colors.push_back(RGB(0xFF, 0x6E, 0xC7));
    colors.push_back(RGB(0x4D, 0x4D, 0xFF));
    colors.push_back(RGB(0x23, 0x23, 0x8E));
    colors.push_back(RGB(0x2F, 0x2F, 0x4F));
    colors.push_back(RGB(0xA6, 0x80, 0x64));
    colors.push_back(RGB(0xDB, 0x70, 0x93));
    colors.push_back(RGB(0x70, 0xDB, 0xDB));
    colors.push_back(RGB(0x7F, 0xFF, 0x00));
    colors.push_back(RGB(0x7F, 0x00, 0xFF));
    colors.push_back(RGB(0x42, 0x6F, 0x42));
    colors.push_back(RGB(0x93, 0x70, 0xDB));
    colors.push_back(RGB(0xEA, 0xEA, 0xAE));
    colors.push_back(RGB(0x6B, 0x8E, 0x23));
    colors.push_back(RGB(0x32, 0x32, 0xCD));
    colors.push_back(RGB(0x32, 0xCD, 0x99));
    colors.push_back(RGB(0x8E, 0x23, 0x6B));
    colors.push_back(RGB(0xE4, 0x78, 0x33));
    colors.push_back(RGB(0xFF, 0x00, 0xFF));
    colors.push_back(RGB(0x32, 0xCD, 0x32));
    colors.push_back(RGB(0xE9, 0xC2, 0xA6));
    colors.push_back(RGB(0x8F, 0x8F, 0xBD));
    colors.push_back(RGB(0xA8, 0xA8, 0xA8));
    colors.push_back(RGB(0xC0, 0xD9, 0xD9));
    colors.push_back(RGB(0x9F, 0x9F, 0x5F));
    colors.push_back(RGB(0x4E, 0x2F, 0x2F));
    colors.push_back(RGB(0x21, 0x5E, 0x21));
    colors.push_back(RGB(0x93, 0xDB, 0x70));
    colors.push_back(RGB(0x52, 0x7F, 0x76));
    colors.push_back(RGB(0x00, 0xFF, 0x00));
    colors.push_back(RGB(0xC0, 0xC0, 0xC0));
    colors.push_back(RGB(0xDB, 0xDB, 0x70));
    colors.push_back(RGB(0xCD, 0x7F, 0x32));
    colors.push_back(RGB(0x23, 0x8E, 0x23));
    colors.push_back(RGB(0x8E, 0x23, 0x23));
    colors.push_back(RGB(0xD1, 0x92, 0x75));
    colors.push_back(RGB(0x85, 0x63, 0x63));
    colors.push_back(RGB(0x54, 0x54, 0x54));
    colors.push_back(RGB(0x85, 0x5E, 0x42));
    colors.push_back(RGB(0x70, 0x93, 0xDB));
    colors.push_back(RGB(0x97, 0x69, 0x4F));
    colors.push_back(RGB(0x2F, 0x4F, 0x4F));
    colors.push_back(RGB(0x6B, 0x23, 0x8E));
    colors.push_back(RGB(0x87, 0x1F, 0x78));
    colors.push_back(RGB(0x99, 0x32, 0xCD));
    colors.push_back(RGB(0x4F, 0x4F, 0x2F));
    colors.push_back(RGB(0x4A, 0x76, 0x6E));
    colors.push_back(RGB(0x2F, 0x4F, 0x2F));
    colors.push_back(RGB(0x5C, 0x40, 0x33));
    colors.push_back(RGB(0x00, 0xFF, 0xFF));
    colors.push_back(RGB(0x42, 0x42, 0x6F));
    colors.push_back(RGB(0xFF, 0x7F, 0x00));
    colors.push_back(RGB(0xB8, 0x73, 0x33));
    colors.push_back(RGB(0xD9, 0x87, 0x19));
    colors.push_back(RGB(0x5F, 0x9F, 0x9F));
    colors.push_back(RGB(0xA6, 0x7D, 0x3D));
    colors.push_back(RGB(0x8C, 0x78, 0x53));
    colors.push_back(RGB(0xA6, 0x2A, 0x2A));
    colors.push_back(RGB(0xD9, 0xD9, 0x19));
    colors.push_back(RGB(0xB5, 0xA6, 0x42));
    colors.push_back(RGB(0x9F, 0x5F, 0x9F));
    colors.push_back(RGB(0x00, 0x00, 0xFF));
    colors.push_back(RGB(0x5C, 0x33, 0x17));
    colors.push_back(RGB(0x70, 0xDB, 0x93));
  }


  RGB
  getHeatColour (double gray)
  {
    uint8_t r, g, b;
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
    RGB colour;
    colour.r = r; colour.g = g; colour.b = b;
    return colour;
  }


  void
  randomColours (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
                 PlanarSegment::StdVectorPtr planar_patches,
                 double min_area,
                 bool project2plane)
  {
    std::vector<RGB> colours;
    getColors(colours);
    output->points.clear();
    output->points.resize(cloud->size());

    srand ( time(NULL) );
    int cnt = 0;
    for (PlanarSegment::StdVector::iterator it = planar_patches->begin(); it != planar_patches->end(); it++)
    {
      //filter out noisy segments
      if (it->area < min_area)
        continue;

      int gray = 255;
      int color_index;
      do{
        color_index = rand() % colours.size();
        gray = (colours[color_index].r + colours[color_index].g + colours[color_index].b) / 3;
      }
      while (gray < 50 || gray > 200);
      if (!project2plane)
      {
        for (int j = 0; j < it->points.size(); j++)
        {
          int index = it->points[j];
          output->points[cnt].x = cloud->points[index].x;
          output->points[cnt].y = cloud->points[index].y;
          output->points[cnt].z = cloud->points[index].z;
          output->points[cnt].r = colours[color_index].r;
          output->points[cnt].g = colours[color_index].g;
          output->points[cnt].b = colours[color_index].b;
          cnt ++;
        }
      }
      else
      {
        for (int j = 0; j < it->points.size(); j++)
        {
          int index = it->points[j];
          double dis = cloud->points[index].x * it->normal(0) +
                       cloud->points[index].y * it->normal(1) +
                       cloud->points[index].z * it->normal(2) - it->bias;
          output->points[cnt].x = cloud->points[index].x - dis * it->normal(0);
          output->points[cnt].y = cloud->points[index].y - dis * it->normal(1);
          output->points[cnt].z = cloud->points[index].z - dis * it->normal(2);
          output->points[cnt].r = colours[color_index].r;
          output->points[cnt].g = colours[color_index].g;
          output->points[cnt].b = colours[color_index].b;
          cnt ++;
        }
      }
    }
    output->points.erase(output->points.begin() + cnt, output->points.end());
    output->height = 1;
    output->width = cnt;
    output->resize(cnt);
  }

  bool
  gerRPYFromRotationMatrix(Eigen::Matrix3d rotation,
                           double &roll,
                           double &pitch,
                           double &yaw,
                           double tolerance)
  {
    bool ret= true;
    pitch = yaw = roll = 0.0;
    //case FiAx:
    pitch = asin(-rotation(2,0)); // value is in range [-Pi/2, Pi/2]
    if (fabs(fabs(pitch)- (M_PI/2.0)) <= tolerance)
    {
      ret= false;
    }
    else{
      double cpitch= cos(pitch);
      roll = atan2(rotation(2,1)/cpitch, rotation(2,2)/cpitch);
      yaw  = atan2(rotation(1,0)/cpitch, rotation(0,0)/cpitch);
    }
    return ret;
  }

}
