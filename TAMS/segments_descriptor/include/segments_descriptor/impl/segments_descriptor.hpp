#ifndef SEGMENTS_DESCRIPTOR_IMPL_H_
#define SEGMENTS_DESCRIPTOR_IMPL_H_
#include <vtkConeSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>

#include "segments_descriptor/segments_descriptor.h"
namespace tams
{
  template <typename PointT> void
  SegmentsDescriptor<PointT> :: orientation_invariant()
  {
    noisy_segments_.clear ();
    saliency_segments_.clear ();
    for (size_t i = 0; i < segments_.size (); i++)
    {
      if (segments_[i].area < min_planar_patch_area_)
      //if (segments_[i].mse * 1000000/(segments_[i].point_num * segments_[i].area) > 20 || segments_[i].area < min_planar_patch_area_)
        noisy_segments_.push_back (segments_[i]);
      else
        saliency_segments_.push_back (segments_[i]);
    }
    //PCL_INFO ("There is/are %d noisy segments with area smaller than 0.25 m*m.\n", noisy_segments_.size());
    PCL_INFO ("There is/are %d saliency segments with area bigger than 0.25 m*m.\n", saliency_segments_.size());
    std::vector<int> z_pp_indices;
    z_pp_indices.clear ();
    for (size_t i = 0; i < saliency_segments_.size (); i++)
    {
      if (saliency_segments_[i].normal.dot(-Vector3d::UnitZ())>0.9850)
        z_pp_indices.push_back(i);
    }
    //PCL_INFO ("%d planar patches found to be almost point to - z axis.\n", z_pp_indices.size ());
    double max = 0;
    int max_index = -1;
    for (std::vector<int>::iterator it = z_pp_indices.begin(); it != z_pp_indices.end(); it++)
    {
      if (saliency_segments_[*it].area > max)
      {
        max = saliency_segments_[*it].area;
        max_index = *it;
      }
    }
    Vector3d axis_1 = Vector3d::Zero();
    double angle_1 = 0.0;
    AngleAxis<double> rotation_1;
    Matrix3d rotation_matrix_1 = Matrix3d::Zero();
    axis_1 = saliency_segments_[max_index].normal.cross (-Vector3d::UnitZ ());
    axis_1 = axis_1.normalized();
    angle_1 = acos (saliency_segments_[max_index].normal.dot (-Vector3d::UnitZ ()));
    rotation_1 = AngleAxis<double>(angle_1, axis_1);
    rotation_matrix_1 = rotation_1.toRotationMatrix ();
    if ((rotation_matrix_1 * saliency_segments_[max_index].normal).dot(-Vector3d::UnitZ()) < 0.9999)
    {
      rotation_1 = AngleAxis<double>(-angle_1, axis_1);
      rotation_matrix_1 = rotation_1.toRotationMatrix ();
    }
    for (size_t i = 0; i < saliency_segments_.size (); i++)
    {
       saliency_segments_[i].normal = rotation_matrix_1 * saliency_segments_[i].normal;
    }
    //PCL_INFO("(%f,%f,%f)\n", saliency_segments_[max_index].normal(0), saliency_segments_[max_index].normal(1), saliency_segments_[max_index].normal(2));
    std::vector<int> z_perpendicular_indices;
    z_perpendicular_indices.clear();
    for (size_t i = 0; i < saliency_segments_.size (); i++)
    {
      if (fabs(saliency_segments_[i].normal.dot(Vector3d::UnitZ())) < 0.17)
      {
        z_perpendicular_indices.push_back(i);
      }
    }
    //PCL_INFO ("%d planar patches found to be almost perpendicular to +/-z axis.\n", z_perpendicular_indices.size ());
    max = 0;
    max_index = -1;
    for (std::vector<int>::iterator it = z_perpendicular_indices.begin(); it != z_perpendicular_indices.end(); it++)
    {
      if (segments_[*it].area > max)
      {
        max = segments_[*it].area;
        max_index = *it;
      }
    }
    Vector3d axis_2 = Vector3d::UnitZ();
    Vector3d tmp = saliency_segments_[max_index].normal;
    tmp(2) = 0;
    tmp = tmp.normalized();
    double angle_2 = acos (tmp.dot (Vector3d::UnitY ()));
    AngleAxis<double> rotation_2 = AngleAxis<double>(angle_2, axis_2);
    Matrix3d rotation_matrix_2 = rotation_2.toRotationMatrix();
    if ((rotation_matrix_2 * tmp).dot(Vector3d::UnitY()) < 0.9999)
    {
      rotation_2 = AngleAxis<double>(-angle_2, axis_2);
      rotation_matrix_2 = rotation_2.toRotationMatrix ();
    }
    for (size_t i = 0; i < saliency_segments_.size (); i++)
    {
       saliency_segments_[i].normal = rotation_matrix_2 * saliency_segments_[i].normal;
    }
// PCL_INFO("(%f,%f,%f)\n", saliency_segments_[max_index].normal(0), saliency_segments_[max_index].normal(1), saliency_segments_[max_index].normal(2));
//    for (size_t i = 0; i < segments_.size (); i++)
//    {
//      std::cout << "normal: (" << segments_[i].normal(0) << ", " << segments_[i].normal(1) << ", " << segments_[i].normal(2) <<
//          "); bias:" << segments_[i].bias <<
//          "; mse:" << segments_[i].mse <<
//          "; area: " << segments_[i].area << ".\n";
//      //"points: " << segments[i].point_num << "; mass_center: ("<<
//      //segments[i].mass_center(0) << ", " << segments[i].mass_center(1) << ", " << segments[i].mass_center(2) << ").\n";
//    }

  }
  /** \todo deal with such situation: only one cluster but contains multiple planar patches.*/
  template <typename PointT> void
  SegmentsDescriptor<PointT> :: clustering ()
  {
    area_at_directions_.clear();
    area_at_directions_.resize(orientations_.size ());
    for (size_t i = 0; i < orientations_.size (); i++)
    {
      area_at_directions_[i].second = orientations_[i];
      area_at_directions_[i].first = 0.0;
    }

    double max, max_index, dot_product;
    for (size_t i = 0; i < saliency_segments_.size (); i++)
    {
      max = -1000.0;
      max_index = -1;
      for (size_t j = 0; j < area_at_directions_.size() - 18; j++)
      {
        dot_product = fabs(saliency_segments_[i].normal.dot (area_at_directions_[j].second));
        //dot_product = saliency_segments_[i].normal.dot (area_at_directions_[j].second);
        if (dot_product > max)
        {
          max = dot_product;
          max_index = j;
        }
      }
      area_at_directions_[max_index].first += saliency_segments_[i].area * saliency_segments_[i].bias;
    }
    for (size_t i = 55; i < 73; i++)
      area_at_directions_[i].first = area_at_directions_[i - 18].first;

//    for (size_t i = 0; i < area_at_directions_.size(); i++)
//    {
//      std::cout << i << "\t" << area_at_directions_[i].first << std::endl;
//    }

    for (size_t i = 1; i < area_at_directions_.size (); i++)
    {
      for (size_t j = i; j < area_at_directions_.size (); j++)
      {
        if (area_at_directions_[i].first < area_at_directions_[j].first)
        {
          Vector3d direction = area_at_directions_[i].second;
          double area = area_at_directions_[i].first;
          area_at_directions_[i].second = area_at_directions_[j].second;
          area_at_directions_[i].first = area_at_directions_[j].first;
          area_at_directions_[j].second = direction;
          area_at_directions_[j].first = area;
        }
      }
    }

  }

  template <typename PointT> void
  SegmentsDescriptor<PointT> :: computeAppearances(std::vector<FeatureBasedOnSegment> &appearances)
  {
    appearances.clear();
    orientation_invariant();
    clustering ();
    size_t primary_index = 1;
    size_t second_primary_index = -1;
    for (size_t i = 1; i < area_at_directions_.size (); i++)
    {
      if (area_at_directions_[i].first / area_at_directions_[1].first < alpha_)
      {
        second_primary_index = i;
        break;
      }
    }
    if (second_primary_index == -1)
      return;
    //std::cout << "primary index:" << primary_index << "\tsecond primary index: " << second_primary_index << std::endl;
    for (size_t j = primary_index; j < second_primary_index; j++)
    {
      FeatureBasedOnSegment feature;
      feature.orientation_hist.clear ();
      feature.orientation_hist.resize(orientations_.size(), 0.0);
      Vector3d axis_1 = Vector3d::Zero();
      double angle_1 = 0.0;
      AngleAxis<double> rotation_1;
      Matrix3d rotation_matrix_1 = Matrix3d::Zero();
      Vector3d axis_2 = Vector3d::Zero();
      double angle_2 = 0.0;
      AngleAxis<double> rotation_2;
      Matrix3d rotation_matrix_2 = Matrix3d::Zero();

      rotation_matrix_1 = Matrix3d::Identity();
      //std::cout << rotation_matrix_1 << std::endl;
      Vector3d tmp_direction = area_at_directions_[j].second;
      tmp_direction(2) = 0.0;
      tmp_direction = tmp_direction.normalized ();
      if (tmp_direction.dot(Vector3d::UnitX()) > 0.9962)
      {
        rotation_matrix_2 = Matrix3d::Identity();
      }
      else
      {
        axis_2 = Vector3d::UnitZ();
        angle_2 = acos (tmp_direction.dot(Vector3d::UnitX ()));
        rotation_2 = AngleAxis<double>(angle_2, axis_2);
        rotation_matrix_2 = rotation_2.toRotationMatrix ();
        if ((rotation_matrix_2 * tmp_direction).dot(Vector3d::UnitX()) < 0.9962)
        {
          rotation_2 = AngleAxis<double>(-angle_2, axis_2);
          rotation_matrix_2 = rotation_2.toRotationMatrix();
        }
      }
      Matrix3d rotation_matrix = rotation_matrix_2 * rotation_matrix_1;
      for (size_t k = 0; k < area_at_directions_.size (); k++)
      {
        Vector3d direction = rotation_matrix * area_at_directions_[k].second;
        double max = -100;
        size_t max_index = -1;
        double dot_product;
        for (size_t q = 0; q < orientations_.size(); q++)
        {
          dot_product = direction.dot(orientations_[q]);
          if (dot_product > max)
          {
            max = dot_product;
            max_index = q;
          }
        }
        feature.volume = volume_;
        feature.normalized_volume = normalized_volume_;
        feature.orientation_hist[max_index] = area_at_directions_[k].first;
        //std::cout << max << "\t" << max_index << "\t" << area_at_directions_[k].first << std::endl;
      }
      appearances.push_back(feature);
    }
    PCL_INFO ("%d appearance(s) constructed.\n", appearances.size());
  }
  template <typename PointT> void
  SegmentsDescriptor <PointT> :: appearancesVisualization(pcl::visualization::PCLVisualizer *pViewer,
                                                          std::vector<FeatureBasedOnSegment> &appearances)
  {
    for (size_t i = 0; i < appearances[0].orientation_hist.size(); i++)
    {
      std::ostringstream s;
      s << "Line" << "%d" << i;
      pViewer -> addLine (pcl::PointXYZ(0.0, 0.0, 0.0),
                          pcl::PointXYZ(orientations_[i](0), orientations_[i](1), orientations_[i](2)),
                          0.0, 1.0, 1.0,
                          s.str());
      vtkSmartPointer<vtkConeSource> coneSource = vtkSmartPointer<vtkConeSource>::New();
      coneSource->SetDirection(orientations_[i](0),
                               orientations_[i](1),
                               orientations_[i](2));
      coneSource->SetHeight(appearances[0].orientation_hist[i] / appearances[0].orientation_hist[0]/4);
      //coneSource->SetCenter(0.0, 0.0, 0.0);
      coneSource->SetCenter(-orientations_[i](0)*coneSource->GetHeight()/2,
                            -orientations_[i](1)*coneSource->GetHeight()/2,
                            -orientations_[i](2)*coneSource->GetHeight()/2);
      coneSource->SetResolution(100);
      coneSource->SetAngle(5);
      coneSource->Update();
      vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
      mapper->SetInputConnection(coneSource->GetOutputPort());
      vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
      actor->SetMapper(mapper);
      vtkSmartPointer<vtkRenderWindow> win = pViewer->getRenderWindow();
      vtkSmartPointer<vtkRendererCollection> rens = win->GetRenderers();

      rens->InitTraversal ();
      vtkRenderer* renderer = NULL;
      int i = 1;
      while ((renderer = rens->GetNextItem ()) != NULL)
      {
         renderer->AddActor (actor);
         renderer->Render ();
      }
    }
    //pViewer->addSphere(pcl::PointXYZ(0.0,0.0,0.0), 0.25, "sphere", 0);
  }

}//end of namespace tamrot
#endif
