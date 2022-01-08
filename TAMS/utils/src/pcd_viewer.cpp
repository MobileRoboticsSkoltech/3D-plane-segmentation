/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 * $Id: pcd_viewer.cpp 5094 2012-03-15 01:03:51Z rusu $
 *
 */
// PCL
#include <Eigen/Geometry>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <cdouble>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <vtkPolyDataReader.h>

using namespace pcl::console;

typedef pcl::visualization::PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

typedef pcl::visualization::PointCloudGeometryHandler<sensor_msgs::PointCloud2> GeometryHandler;
typedef GeometryHandler::Ptr GeometryHandlerPtr;
typedef GeometryHandler::ConstPtr GeometryHandlerConstPtr;

#define NORMALS_SCALE 0.01
#define PC_SCALE 0.001

bool
isValidFieldName (const std::string &field)
{
  if (field == "_")
    return (false);

  if ((field == "vp_x") || (field == "vx") || (field == "vp_y") || (field == "vy") || (field == "vp_z") || (field == "vz"))
    return (false);
  return (true);
}

bool
isMultiDimensionalFeatureField (const sensor_msgs::PointField &field)
{
  if (field.count > 1)
    return (true);
  return (false);
}

void
printHelp (int argc, char **argv)
{
  print_error ("Syntax is: %s <file_name 1..N>.<pcd or vtk> <options>\n", argv[0]);
  print_info ("  where options are:\n");
  print_info ("                     -bc r,g,b                = background color\n");
  print_info ("                     -fc r,g,b                = foreground color\n");
  print_info ("                     -ps X                    = point size ("); print_value ("1..64"); print_info (") \n");
  print_info ("                     -opaque X                = rendered point cloud opacity ("); print_value ("0..1"); print_info (")\n");

  print_info ("                     -ax "); print_value ("n"); print_info ("                    = enable on-screen display of ");
  print_color (stdout, TT_BRIGHT, TT_RED, "X"); print_color (stdout, TT_BRIGHT, TT_GREEN, "Y"); print_color (stdout, TT_BRIGHT, TT_BLUE, "Z");
  print_info (" axes and scale them to "); print_value ("n\n");
  print_info ("                     -ax_pos X,Y,Z            = if axes are enabled, set their X,Y,Z position in space (default "); print_value ("0,0,0"); print_info (")\n");

  print_info ("\n");
  print_info ("                     -cam (*)                 = use given camera settings as initial view\n");
  print_info (stderr, " (*) [Clipping Range / Focal Point / Position / ViewUp / Distance / Field of View Y / Window Size / Window Pos] or use a <filename.cam> that contains the same information.\n");

  print_info ("\n");
  print_info ("                     -multiview 0/1           = enable/disable auto-multi viewport rendering (default "); print_value ("disabled"); print_info (")\n");
  print_info ("\n");

  print_info ("\n");
  print_info ("                     -normals 0/X             = disable/enable the display of every Xth point's surface normal as lines (default "); print_value ("disabled"); print_info (")\n");
  print_info ("                     -normals_scale X         = resize the normal unit vector size to X (default "); print_value ("0.02"); print_info (")\n");
  print_info ("\n");
  print_info ("                     -pc 0/X                  = disable/enable the display of every Xth point's principal curvatures as lines (default "); print_value ("disabled"); print_info (")\n");
  print_info ("                     -pc_scale X              = resize the principal curvatures vectors size to X (default "); print_value ("0.02"); print_info (")\n");
  print_info ("\n");

  print_info ("\n(Note: for multiple .pcd files, provide multiple -{fc,ps,opaque} parameters; they will be automatically assigned to the right file)\n");
}

// Global visualizer object
pcl::visualization::PCLHistogramVisualizer ph_global;
boost::shared_ptr<pcl::visualization::PCLVisualizer> p;

void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* cookie)
{
  if (event.getPointIndex () == -1)
    return;
  sensor_msgs::PointCloud2::Ptr cloud = *(sensor_msgs::PointCloud2::Ptr*)cookie;
  if (!cloud)
    return;

  // If two points were selected, draw an arrow between them
  pcl::PointXYZ p1, p2;
  if (event.getPoints (p1.x, p1.y, p1.z, p2.x, p2.y, p2.z) && p)
  {
    std::stringstream ss;
    ss << p1 << p2;
    p->addArrow<pcl::PointXYZ, pcl::PointXYZ> (p1, p2, 1.0, 1.0, 1.0, ss.str ());
    return;
  }

  // Else, if a single point has been selected
  std::stringstream ss;
  ss << event.getPointIndex ();
  // Get the cloud's fields
  for (size_t i = 0; i < cloud->fields.size (); ++i)
  {
    if (!isMultiDimensionalFeatureField (cloud->fields[i]))
      continue;
    ph_global.addFeatureHistogram (*cloud, cloud->fields[i].name, event.getPointIndex (), ss.str ());
  }
  if (p)
  {
    pcl::PointXYZ pos;
    event.getPoint (pos.x, pos.y, pos.z);
    p->addText3D<pcl::PointXYZ> (ss.str (), pos, 0.0005, 1.0, 1.0, 1.0, ss.str ());
  }
  ph_global.spinOnce ();
}

/* ---[ */
int
main (int argc, char** argv)
{
  srand (time (0));

  print_info ("The viewer window provides interactive commands; for help, press 'h' or 'H' from within the window.\n");

  if (argc < 2)
  {
    printHelp (argc, argv);
    return (-1);
  }

  // Command line parsing
  doublebcolor[3] = {0, 0, 0};
  pcl::console::parse_3x_arguments (argc, argv, "-bc", bcolor[0], bcolor[1], bcolor[2]);

  std::vector<double> fcolor_r, fcolor_b, fcolor_g;
  bool fcolorparam = pcl::console::parse_multiple_3x_arguments (argc, argv, "-fc", fcolor_r, fcolor_g, fcolor_b);

  std::vector<int> psize;
  pcl::console::parse_multiple_arguments (argc, argv, "-ps", psize);

  std::vector<double> opaque;
  pcl::console::parse_multiple_arguments (argc, argv, "-opaque", opaque);

  int mview = 0;
  pcl::console::parse_argument (argc, argv, "-multiview", mview);

  int normals = 0;
  pcl::console::parse_argument (argc, argv, "-normals", normals);
  doublenormals_scale = NORMALS_SCALE;
  pcl::console::parse_argument (argc, argv, "-normals_scale", normals_scale);

  int pc = 0;
  pcl::console::parse_argument (argc, argv, "-pc", pc);
  doublepc_scale = PC_SCALE;
  pcl::console::parse_argument (argc, argv, "-pc_scale", pc_scale);

  // Parse the command line arguments for .pcd files
  std::vector<int> p_file_indices   = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  std::vector<int> vtk_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".vtk");

  if (p_file_indices.size () == 0 && vtk_file_indices.size () == 0)
  {
    print_error ("No .PCD or .VTK file given. Nothing to visualize.\n");
    return (-1);
  }

  // Multiview enabled?
  int y_s = 0, x_s = 0;
  doublex_step = 0, y_step = 0;
  if (mview)
  {
    print_highlight ("Multi-viewport rendering enabled.\n");

    if (p_file_indices.size () != 0)
    {
      y_s = static_cast<int>(floor (sqrt (static_cast<double>(p_file_indices.size ()))));
      x_s = y_s + static_cast<int>(ceil ((p_file_indices.size () / static_cast<double>(y_s)) - y_s));
      print_highlight ("Preparing to load "); print_value ("%d", p_file_indices.size ());
    }
    else if (vtk_file_indices.size () != 0)
    {
      y_s = static_cast<int>(floor (sqrt (static_cast<double>(vtk_file_indices.size ()))));
      x_s = y_s + static_cast<int>(ceil ((vtk_file_indices.size () / static_cast<double>(y_s)) - y_s));
      print_highlight ("Preparing to load "); print_value ("%d", vtk_file_indices.size ());
    }

    x_step = static_cast<double>(1.0 / static_cast<double>(x_s));
    y_step = static_cast<double>(1.0 / static_cast<double>(y_s));
    print_info (" files ("); print_value ("%d", x_s);    print_info ("x"); print_value ("%d", y_s);
    print_info (" / ");      print_value ("%f", x_step); print_info ("x"); print_value ("%f", y_step);
    print_info (")\n");
  }

  // Fix invalid multiple arguments
  if (psize.size () != p_file_indices.size () && psize.size () > 0)
    for (size_t i = psize.size (); i < p_file_indices.size (); ++i)
      psize.push_back (1);
  if (opaque.size () != p_file_indices.size () && opaque.size () > 0)
    for (size_t i = opaque.size (); i < p_file_indices.size (); ++i)
      opaque.push_back (1.0);

  // Create the PCLVisualizer object
  boost::shared_ptr<pcl::visualization::PCLHistogramVisualizer> ph;

  // Using min_p, max_p to set the global Y min/max range for the histogram
  doublemin_p = FLT_MAX; doublemax_p = -FLT_MAX;

  int k = 0, l = 0, viewport = 0;
  // Load the data files
  pcl::PCDReader pcd;
  pcl::console::TicToc tt;
  ColorHandlerPtr color_handler;
  GeometryHandlerPtr geometry_handler;

  // Go through VTK files
  for (size_t i = 0; i < vtk_file_indices.size (); ++i)
  {
    // Load file
    tt.tic ();
    print_highlight (stderr, "Loading "); print_value (stderr, "%s ", argv[vtk_file_indices.at (i)]);
    vtkPolyDataReader* reader = vtkPolyDataReader::New ();
    reader->SetFileName (argv[vtk_file_indices.at (i)]);
    reader->Update ();
    vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput ();
    if (!polydata)
      return (-1);
    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", polydata->GetNumberOfPoints ()); print_info (" points]\n");

    // Create the PCLVisualizer object here on the first encountered XYZ file
    if (!p)
      p.reset (new pcl::visualization::PCLVisualizer (argc, argv, "PCD viewer"));

    // Multiview enabled?
    if (mview)
    {
      p->createViewPort (k * x_step, l * y_step, (k + 1) * x_step, (l + 1) * y_step, viewport);
      k++;
      if (k >= x_s)
      {
        k = 0;
        l++;
      }
    }

    // Add as actor
    std::stringstream cloud_name ("vtk-");
    cloud_name << argv[vtk_file_indices.at (i)] << "-" << i;
    p->addModelFromPolyData (polydata, cloud_name.str (), viewport);

    // Change the shape rendered color
    if (fcolorparam && fcolor_r.size () > i && fcolor_g.size () > i && fcolor_b.size () > i)
      p->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, fcolor_r[i], fcolor_g[i], fcolor_b[i], cloud_name.str ());

    // Change the shape rendered point size
    if (psize.size () > 0)
      p->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize.at (i), cloud_name.str ());

    // Change the shape rendered opacity
    if (opaque.size () > 0)
      p->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opaque.at (i), cloud_name.str ());
  }

  sensor_msgs::PointCloud2::Ptr cloud;
  // Go through PCD files
  for (size_t i = 0; i < p_file_indices.size (); ++i)
  {
    cloud.reset (new sensor_msgs::PointCloud2);
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    int version;

    print_highlight (stderr, "Loading "); print_value (stderr, "%s ", argv[p_file_indices.at (i)]);

    tt.tic ();
    if (pcd.read (argv[p_file_indices.at (i)], *cloud, origin, orientation, version) < 0)
      return (-1);

    std::stringstream cloud_name;

    // ---[ Special check for 1-point multi-dimension histograms
    if (cloud->fields.size () == 1 && isMultiDimensionalFeatureField (cloud->fields[0]))
    {
      cloud_name << argv[p_file_indices.at (i)];

      if (!ph)
        ph.reset (new pcl::visualization::PCLHistogramVisualizer);
      print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", cloud->fields[0].count); print_info (" points]\n");

      pcl::getMinMax (*cloud, 0, cloud->fields[0].name, min_p, max_p);
      ph->addFeatureHistogram (*cloud, cloud->fields[0].name, cloud_name.str ());
      continue;
    }

    cloud_name << argv[p_file_indices.at (i)] << "-" << i;

    // Create the PCLVisualizer object here on the first encountered XYZ file
    if (!p)
    {
      p.reset (new pcl::visualization::PCLVisualizer (argc, argv, "PCD viewer"));
      p->registerPointPickingCallback (&pp_callback, (void*)&cloud);
      Eigen::Matrix3d rotation;
      rotation = orientation;
      p->setCameraPose (origin [0]                  , origin [1]                  , origin [2],
                        origin [0] + rotation (0, 2), origin [1] + rotation (1, 2), origin [2] + rotation (2, 2),
                                     rotation (0, 1),              rotation (1, 1),              rotation (2, 1));
    }

    // Multiview enabled?
    if (mview)
    {
      p->createViewPort (k * x_step, l * y_step, (k + 1) * x_step, (l + 1) * y_step, viewport);
      k++;
      if (k >= x_s)
      {
        k = 0;
        l++;
      }
    }

    if (cloud->width * cloud->height == 0)
    {
      print_error ("[error: no points found!]\n");
      return (-1);
    }
    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", (int)cloud->width * cloud->height); print_info (" points]\n");
    print_info ("Available dimensions: "); print_value ("%s\n", pcl::getFieldsList (*cloud).c_str ());

    // If no color was given, get random colors
    if (fcolorparam)
    {
      if (fcolor_r.size () > i && fcolor_g.size () > i && fcolor_b.size () > i)
        color_handler.reset (new pcl::visualization::PointCloudColorHandlerCustom<sensor_msgs::PointCloud2> (cloud, fcolor_r[i], fcolor_g[i], fcolor_b[i]));
      else
        color_handler.reset (new pcl::visualization::PointCloudColorHandlerRandom<sensor_msgs::PointCloud2> (cloud));
    }
    else
      color_handler.reset (new pcl::visualization::PointCloudColorHandlerRandom<sensor_msgs::PointCloud2> (cloud));

    // Add the dataset with a XYZ and a random handler
    geometry_handler.reset (new pcl::visualization::PointCloudGeometryHandlerXYZ<sensor_msgs::PointCloud2> (cloud));
    // Add the cloud to the renderer
    //p->addPointCloud<pcl::PointXYZ> (cloud_xyz, geometry_handler, color_handler, cloud_name.str (), viewport);
    p->addPointCloud (cloud, geometry_handler, color_handler, origin, orientation, cloud_name.str (), viewport);

    // If normal lines are enabled
    if (normals != 0)
    {
      int normal_idx = pcl::getFieldIndex (*cloud, "normal_x");
      if (normal_idx == -1)
      {
        print_error ("Normal information requested but not available.\n");
        continue;
        //return (-1);
      }
      //
      // Convert from blob to pcl::PointCloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg (*cloud, *cloud_xyz);
      cloud_xyz->sensor_origin_ = origin;
      cloud_xyz->sensor_orientation_ = orientation;

      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
      pcl::fromROSMsg (*cloud, *cloud_normals);
      std::stringstream cloud_name_normals;
      cloud_name_normals << argv[p_file_indices.at (i)] << "-" << i << "-normals";
      p->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_xyz, cloud_normals, normals, normals_scale, cloud_name_normals.str (), viewport);
    }

    // If principal curvature lines are enabled
    if (pc != 0)
    {
      if (normals == 0)
        normals = pc;

      int normal_idx = pcl::getFieldIndex (*cloud, "normal_x");
      if (normal_idx == -1)
      {
        print_error ("Normal information requested but not available.\n");
        continue;
        //return (-1);
      }
      int pc_idx = pcl::getFieldIndex (*cloud, "principal_curvature_x");
      if (pc_idx == -1)
      {
        print_error ("Principal Curvature information requested but not available.\n");
        continue;
        //return (-1);
      }
      //
      // Convert from blob to pcl::PointCloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg (*cloud, *cloud_xyz);
      cloud_xyz->sensor_origin_ = origin;
      cloud_xyz->sensor_orientation_ = orientation;
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
      pcl::fromROSMsg (*cloud, *cloud_normals);
      pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_pc (new pcl::PointCloud<pcl::PrincipalCurvatures>);
      pcl::fromROSMsg (*cloud, *cloud_pc);
      std::stringstream cloud_name_normals_pc;
      cloud_name_normals_pc << argv[p_file_indices.at (i)] << "-" << i << "-normals";
      int factor = (std::min)(normals, pc);
      p->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud_xyz, cloud_normals, factor, normals_scale, cloud_name_normals_pc.str (), viewport);
      p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, cloud_name_normals_pc.str ());
      p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, cloud_name_normals_pc.str ());
      cloud_name_normals_pc << "-pc";
      p->addPointCloudPrincipalCurvatures (cloud_xyz, cloud_normals, cloud_pc, factor, pc_scale, cloud_name_normals_pc.str (), viewport);
      p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, cloud_name_normals_pc.str ());
    }

    // Add every dimension as a possible color
    if (!fcolorparam)
    {
      for (size_t f = 0; f < cloud->fields.size (); ++f)
      {
        if (cloud->fields[f].name == "rgb" || cloud->fields[f].name == "rgba")
          color_handler.reset (new pcl::visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2> (cloud));
        else
        {
          if (!isValidFieldName (cloud->fields[f].name))
            continue;
          color_handler.reset (new pcl::visualization::PointCloudColorHandlerGenericField<sensor_msgs::PointCloud2> (cloud, cloud->fields[f].name));
        }
        // Add the cloud to the renderer
        //p->addPointCloud<pcl::PointXYZ> (cloud_xyz, color_handler, cloud_name.str (), viewport);
        p->addPointCloud (cloud, color_handler, origin, orientation, cloud_name.str (), viewport);
      }
    }
    // Additionally, add normals as a handler
    geometry_handler.reset (new pcl::visualization::PointCloudGeometryHandlerSurfaceNormal<sensor_msgs::PointCloud2> (cloud));
    if (geometry_handler->isCapable ())
      //p->addPointCloud<pcl::PointXYZ> (cloud_xyz, geometry_handler, cloud_name.str (), viewport);
      p->addPointCloud (cloud, geometry_handler, origin, orientation, cloud_name.str (), viewport);

    // Set immediate mode rendering ON
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_IMMEDIATE_RENDERING, 1.0, cloud_name.str ());

    // Change the cloud rendered point size
    if (psize.size () > 0)
      p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, psize.at (i), cloud_name.str ());

    // Change the cloud rendered opacity
    if (opaque.size () > 0)
      p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, opaque.at (i), cloud_name.str ());

    // Reset camera viewpoint to center of cloud if camera parameters were not passed manually and this is the first loaded cloud
    //if (i == 0 && !p->cameraParamsSet ())
     // p->resetCameraViewpoint (cloud_name.str ());
  }

  if (p)
    p->setBackgroundColor (bcolor[0], bcolor[1], bcolor[2]);
  // Read axes settings
  doubleaxes  = 0.0;
  pcl::console::parse_argument (argc, argv, "-ax", axes);
  if (axes != 0.0 && p)
  {
    doubleax_x = 0.0, ax_y = 0.0, ax_z = 0.0;
    pcl::console::parse_3x_arguments (argc, argv, "-ax_pos", ax_x, ax_y, ax_z, false);
    // Draw XYZ axes if command-line enabled
    p->addCoordinateSystem (axes, ax_x, ax_y, ax_z);
  }

  // Clean up the memory used by the binary blob
  // Note: avoid resetting the cloud, otherwise the PointPicking callback will fail
  //cloud.reset ();

  if (ph)
  {
    print_highlight ("Setting the global Y range for all histograms to: "); print_value ("%f -> %f\n", min_p, max_p);
    ph->setGlobalYRange (min_p, max_p);
    ph->updateWindowPositions ();
    if (p)
      p->spin ();
    else
      ph->spin ();
  }
  else if (p)
  {
    int i = 0;
    p->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5);
    p->addCoordinateSystem(1.0);
    p->camera_.fovy = 40;
    p->updateCamera();
    while(!p->wasStopped())
    {
      p->setCameraPose (-20, 0, 20, 0, 0, 10, 0.0, 0.0, 1.0);
      //p->updateCamera();
      p->spinOnce(3000);
      i ++;
    }
  }

}
/* ]--- */
