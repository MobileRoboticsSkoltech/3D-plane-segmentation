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

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include <pcl/io/ply_io.h>

int
main (int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "usage: boundingbox <input_pcd> <output_ply>" << std::endl;
    }
    std::string input_pcd(argv[1]);
    pcl::PointCloud<pcl::PointXYZ> input;
    std::string output_ply(argv[2]);

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (input_pcd, input) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file %s!\n", input_pcd.c_str());
      return (-1);
    }
    PCL_INFO ("Loaded %d points from %s.\n", input.points.size(), input_pcd.c_str ());

    if (input.height != 1)
    {
      std::cerr << "the input is an organized point cloud.\n";
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(input, input, indices);
    }
    else
    {
      std::cerr << "the input is an unorganized point cloud.\n";
    }


    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    double mean_x = 0.0, mean_y = 0.0, mean_z = 0.0;
    double max_x = -9999, min_x = 9999, max_y = -9999, min_y = 9999, max_z = -9999, min_z = 9999;
    for (size_t i = 0; i < input.points.size(); i++)
    {
      sum_x += input.points[i].x;
      sum_y += input.points[i].y;
      sum_z += input.points[i].z;

      if (input.points[i].x < min_x)
        min_x = input.points[i].x;
      if (input.points[i].x > max_x)
        max_x = input.points[i].x;

      if (input.points[i].y < min_y)
        min_y = input.points[i].y;
      if (input.points[i].y > max_y)
        max_y = input.points[i].y;

      if (input.points[i].z < min_z)
        min_z = input.points[i].z;
      if (input.points[i].z > max_z)
        max_z = input.points[i].z;
    }

    mean_x = sum_x / input.points.size ();
    mean_y = sum_y / input.points.size ();
    mean_z = sum_z / input.points.size ();

    for (size_t i = 0; i < input.points.size(); i++)
    {
      input.points[i].x = input.points[i].x - mean_x;
      input.points[i].y = input.points[i].y - mean_y;
      input.points[i].z = input.points[i].z - mean_z;
    }

    max_x = max_x - mean_x;
    min_x = min_x - mean_x;

    max_y = max_y - mean_y;
    min_y = min_y - mean_y;

    max_z = max_z - mean_z;
    min_z = min_z - mean_z;

    pcl::PLYWriter ply_writer;
    ply_writer.write (output_ply.c_str (), input);

    std::cerr << "bounding box <min_x max_y min_y max_y min_z max_z>: "
              << min_x << " " << max_x << " "
              << min_y << " " << max_y << " "
              << min_z << " " << max_z << std::endl;

    return (0);
}
