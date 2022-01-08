#ifndef HYBRID_REGION_GROWING_SEGMENTATION_IMPL_H_
#define HYBRID_REGION_GROWING_SEGMENTATION_IMPL_H_
#include "region_growing_segmentation/region_growing_segmentation.h"
#include "common/rgb.h"
#include <algorithm>
namespace tams
{
  template <typename PointT> double
  RGSegmentation<PointT>::computeRightBottomArea(const int pos_index, PlanarSegment &segment)
  {
    ///pos_index - width -1     pos_index - 1  pos_index -1 + width
    ///pos_index - width        pos_index      pos_index + width
    ///pos_index + 1 - width    pos_index + 1  pos_index + width + 1
    vector<int>::iterator right_it, bottom_it, right_bottom_it;
    right_it = find(segment.points.begin(), segment.points.end(), pos_index + width_);
    bottom_it = find(segment.points.begin(), segment.points.end(), pos_index + 1);
    right_bottom_it = find(segment.points.begin(), segment.points.end(), pos_index + width_ + 1);
    if (right_it == segment.points.end() && bottom_it == segment.points.end() && right_bottom_it == segment.points.end())
      return 0.0;
    if (right_it != segment.points.end() && bottom_it == segment.points.end() && right_bottom_it == segment.points.end())
      return 0.0;
    if (right_it == segment.points.end() && bottom_it != segment.points.end() && right_bottom_it == segment.points.end())
      return 0.0;
    if (right_it == segment.points.end() && bottom_it == segment.points.end() && right_bottom_it != segment.points.end())
      return 0.0;

    if (right_it == segment.points.end() && bottom_it != segment.points.end() && right_bottom_it != segment.points.end())
    {
      return 0.5 * fabs(segment.normal.dot(points_[pos_index].cross(points_[pos_index + 1]) +
                                           points_[pos_index + 1].cross(points_[pos_index + width_ + 1]) +
                                           points_[pos_index + width_ + 1].cross(points_[pos_index])));
    }
    if (right_it != segment.points.end() && bottom_it == segment.points.end() && right_bottom_it != segment.points.end())
    {
      return 0.5 * fabs(segment.normal.dot(points_[pos_index].cross(points_[pos_index + width_]) +
                                           points_[pos_index + width_].cross(points_[pos_index + width_ + 1]) +
                                           points_[pos_index + width_ + 1].cross(points_[pos_index])));
    }
    if (right_it != segment.points.end() && bottom_it != segment.points.end() && right_bottom_it == segment.points.end())
    {
      return 0.5 * fabs(segment.normal.dot(points_[pos_index].cross(points_[pos_index + width_]) +
                                           points_[pos_index + width_].cross(points_[pos_index + 1]) +
                                           points_[pos_index + 1].cross(points_[pos_index])));
    }
    if (right_it != segment.points.end() && bottom_it != segment.points.end() && right_bottom_it != segment.points.end())
    {
      return 0.5 * fabs(segment.normal.dot(points_[pos_index].cross(points_[pos_index + width_]) +
                                           points_[pos_index + width_].cross(points_[pos_index + width_ + 1]) +
                                           points_[pos_index + width_ + 1].cross(points_[pos_index + 1]) +
                                           points_[pos_index + 1].cross(points_[pos_index])));
    }
  }

  template <typename PointT> double
  RGSegmentation<PointT>::computeLeftTopArea(const int pos_index, PlanarSegment &segment)
  {
    ///pos_index - width -1     pos_index - 1  pos_index -1 + width
    ///pos_index - width        pos_index      pos_index + width
    ///pos_index + 1 - width    pos_index + 1  pos_index + width + 1
    vector<int>::iterator left_it, top_it;
    left_it = find(segment.points.begin(), segment.points.end(), pos_index - width_);
    top_it = find(segment.points.begin(), segment.points.end(), pos_index - 1);
    if (left_it == segment.points.end() || top_it == segment.points.end())
      return 0.0;
    return 0.5 * fabs(segment.normal.dot(points_[pos_index].cross(points_[pos_index - width_]) +
                                         points_[pos_index - width_].cross(points_[pos_index - 1]) +
                                         points_[pos_index - 1].cross(points_[pos_index])));
  }

  template <typename PointT> void
  RGSegmentation<PointT>::computeSegmentsArea()
  {
    ///pos_index - width -1     pos_index - 1  pos_index -1 + width
    ///pos_index - width        pos_index      pos_index + width
    ///pos_index + 1 - width    pos_index + 1  pos_index + width + 1
    vector<Vector3d, aligned_allocator<Vector3d> > cross_products;
    cross_products.resize(planar_patches_.size(), Vector3d::Zero());
    int *flags = new int [height_ * width_];
    memset(flags, 9999, height_ * width_ * sizeof(int));
    for (int i = 0; i < static_cast<int>(planar_patches_.size ()); i++)
    {
      planar_patches_[i].area = 0;
      for (vector<int>::iterator it = planar_patches_[i].points.begin(); it != planar_patches_[i].points.end (); it++)
      {
        flags[*it] = i;
      }
    }
    int index = 0;
    int flag = -1;
    for (int i = 1; i < height_ - 1; i++)
    {
      for (int j = 1; j < width_ - 1; j++)
      {
        index = i * width_ + j;
        flag = flags[index];
        if (flag < 9999)
        {
          int key = (flags[index + width_] == flag) * 4 + (flags[index + width_ + 1] == flag) * 2 + (flags[index + 1] == flag);
          switch (key)
          {
            case 7:
              cross_products[flag] +=
              //planar_patches_[flag].area += 0.5 * fabs(planar_patches_[flag].normal.dot(
                                      points_[index].cross(points_[index + width_]) +
                                      points_[index + width_].cross(points_[index + width_ + 1]) +
                                      points_[index + width_ + 1].cross(points_[index + 1]) +
                                      points_[index + 1].cross(points_[index]);//));
              break;
            case 3:
              cross_products[flag] +=
              //planar_patches_[flag].area += 0.5 * fabs(planar_patches_[flag].normal.dot(
                                      points_[index].cross(points_[index + width_ + 1]) +
                                      points_[index + width_ + 1].cross(points_[index + 1]) +
                                      points_[index + 1].cross(points_[index]);//));
              break;
            case 5:
              cross_products[flag] +=
              //planar_patches_[flag].area += 0.5 * fabs(planar_patches_[flag].normal.dot(
                                      points_[index].cross(points_[index + width_]) +
                                      points_[index + width_].cross(points_[index + 1]) +
                                      points_[index + 1].cross(points_[index]);//));
              break;
            case 6:
              cross_products[flag] +=
              //planar_patches_[flag].area += 0.5 * fabs(planar_patches_[flag].normal.dot(
                                      points_[index].cross(points_[index + width_]) +
                                      points_[index + width_].cross(points_[index + width_ + 1]) +
                                      points_[index + width_ + 1].cross(points_[index]);//));
              break;
            default:
              break;
          }
          if ((flags[index - width_ - 1] != flag) && (flags[index - 1] == flag) && (flags[index - width_] == flag))
          {
            cross_products[flag] +=
            //planar_patches_[flag].area += 0.5 * fabs(planar_patches_[flag].normal.dot(
                                    points_[index].cross(points_[index - width_]) +
                                    points_[index - width_].cross(points_[index - 1]) +
                                    points_[index - 1].cross(points_[index]);//));
          }
        }
      }
    }
    for (size_t i = 0; i < planar_patches_.size (); i++)
    {
      planar_patches_[i].area = 0.5 * fabs(planar_patches_[i].normal.dot(cross_products[i]));
//      std::cout << planar_patches_[i].area << std::endl;
    }
    delete [] flags;
    PCL_INFO ("Area computation finished!\n");
  }

/*
  template <typename PointT> void
  RGSegmentation<PointT>::computeSegmentsArea()
  {
    ///pos_index - height -1     pos_index - 1  pos_index -1 + height
    ///pos_index - height        pos_index      pos_index + height
    ///pos_index + 1 - height    pos_index + 1  pos_index + height + 1
    vector<int>::iterator left_top_it;
    int pos_index;
    PointXY pos;
    for (size_t i=0; i < planar_patches_.size(); i++)
    {
      PlanarSegments segment = planar_patches_[i];
      if (static_cast<int>(segment.points.size()) != segment.point_num)
      {
        PCL_ERROR("segment.points.size()) != segment.point_num!\n");
      }
      for (int j = 0; j < segment.point_num; j++)
      {
        pos_index = segment.points[j];
        pos.first = pos_index % width_;
        pos.second = pos_index / width_;
        if (pos.first == 0 || pos.first == width_ - 1 || pos.second == 0 || pos.second == height_ -1)
          continue;
        left_top_it = find(segment.points.begin(), segment.points.end(), pos_index - width_ -1);
        //left top point is not in the same segment
        if (left_top_it == segment.points.end())
        {
          segment.area += computeLeftTopArea(pos_index, segment);
          segment.area += computeRightBottomArea(pos_index, segment);
        }//left top point is in the same segment
        else
        {
          segment.area += computeRightBottomArea(pos_index, segment);
        }
      }
void
Registration::mergeSurfacesOnSameInfinitePlane(double   min_dot_product,
                                               double   max_bias_dif)
{
  tamrot::PlanarSegments pp;
  PlanarSegments::StdVector::iterator it1,it2;
  std::vector<tamrot::PlanarSegments> segments;

  for (it1 = big_map_segments_->begin(); it1 != big_map_segments_->end()-1; it1++)
  {
    if (it1->area == 0)
      continue;
    for (it2 = it1+1; it2 != big_map_segments_->end(); it2++)
    {
      if (it2->area == 0)
        continue;
      if (it2->normal.dot(it1->normal) < min_dot_product)
        continue;
      if (fabs(it2->bias - it1->bias) > max_bias_dif)
        continue;
      it1->normal = it1->normal * it1->area / (it1->area + it2->area) +
                    it2->normal * it2->area / (it1->area + it2->area);
      it1->normal = it1->normal.normalized();
      it1->bias = it1->bias * it1->area / (it1->area + it2->area) +
                  it2->bias * it2->area / (it1->area + it2->area);
      it1->area += it2->area;
      it2->area = 0;
      it2->normal = Vector3d::Zero();
    }
  }
  segments.clear();
  for (it1 = big_map_segments_->begin(); it1 != big_map_segments_->end(); it1++)
  {
    if (it1->area == 0)
      continue;
    segments.push_back(*it1);
  }
  big_map_segments_->clear();
  big_map_segments_->insert(big_map_segments_->begin(), segments.begin(), segments.end());

  for (it1 = big_data_segments_->begin(); it1 != big_data_segments_->end()-1; it1++)
  {
    if (it1->area == 0)
      continue;
    for (it2 = it1+1; it2 != big_data_segments_->end(); it2++)
    {
      if (it2->area == 0)
        continue;
      if (it2->normal.dot(it1->normal) < min_dot_product)
        continue;
      if (fabs(it2->bias - it1->bias) > max_bias_dif)
        continue;
      it1->normal = it1->normal * it1->area / (it1->area + it2->area) +
                    it2->normal * it2->area / (it1->area + it2->area);
      it1->normal = it1->normal.normalized();
      it1->bias = it1->bias * it1->area / (it1->area + it2->area) +
                  it2->bias * it2->area / (it1->area + it2->area);
      it1->area += it2->area;
      it2->area = 0;
      it2->normal = Vector3d::Zero();
    }
  }
  segments.clear();
  for (it1 = big_data_segments_->begin(); it1 != big_data_segments_->end(); it1++)
  {
    if (it1->area == 0)
      continue;
    segments.push_back(*it1);
  }
  big_data_segments_->clear();
  big_data_segments_->insert(big_data_segments_->begin(), segments.begin(), segments.end());

  PCL_INFO ("There are %d and %d segments in the map cloud and data cloud after merging planes, respectively.\n",
            big_map_segments_->size(), big_data_segments_->size());
}

      planar_patches_[i].area = segment.area;
    }
  }
*/

  template <typename PointT> void
  RGSegmentation<PointT>::saveTimes ()
  {
    ofstream times_txt;
    times_txt.open ("times.txt", fstream::app);
    double   t0 = times.begin ()->tv_sec + (times.begin ()->tv_usec / 1000000.0);
    double   t;
    for (vector<timeval>::iterator iter = times.begin () + 1; iter != times.end (); iter++)
    {
      t = iter->tv_sec + (iter->tv_usec / 1000000.0);
      times_txt << t - t0 << " ";
    }
    times_txt << endl;
  }

  template <typename PointT> void
  RGSegmentation<PointT>::slidingWindow(const int sliding_window_size)
  {
    sliding_windows_.clear ();
    SlidingWindowItem tmp;
    Vector3d sum = Vector3d::Zero();
    Matrix3d scatter_matrix = Matrix3d::Zero();
    Vector3d mass_center = Vector3d::Zero();
    EigenSolver<Matrix3d> eigensolver;
    Vector3d eigenvalues = Vector3d::Zero();
    Matrix3d eigenvectors = Matrix3d::Zero();
    vector<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > points;
    int valid_cnt = 0;
    int row = 0, col = 0;
    int grid_size = (2 * sliding_window_size + 1) * (2 * sliding_window_size + 1);
    int *local_indices = new int [grid_size];
    for (int i = sliding_window_size; i < width_ - sliding_window_size; i++)
    {
      for (int j = sliding_window_size; j < height_ - sliding_window_size; j++)
      {
        if (!valid_[j * width_ + i])
          continue;
        memset(local_indices, 0, grid_size);
        sum = Vector3d::Zero();
        scatter_matrix = Matrix3d::Zero();
        points.clear();
        valid_cnt = 0;
        int col_start = i - sliding_window_size;
        int col_end = i + sliding_window_size;
        int row_start = j - sliding_window_size;
        int row_end   = j + sliding_window_size;
        int t = 0;
        for (col = col_start; col <= col_end; col++)
        {
          for (row = row_start; row <= row_end; row++)
          {
            local_indices[t++] = row * width_ + col;
          }
        }
        for (int k = 0; k < grid_size; k++)
        {
          if (valid_[local_indices[k]])
          {
            valid_cnt++;
          }
        }
        if (valid_cnt > grid_size * 0.7)
        {
          for (int k = 0; k < grid_size; k++)
          {
            if (valid_[local_indices[k]])
              sum += points_[local_indices[k]];
          }
          mass_center = sum / valid_cnt;
          for (int k = 0; k < grid_size; k++)
          {
            if (valid_[local_indices[k]])
              scatter_matrix += (points_[local_indices[k]] - mass_center) * (points_[local_indices[k]] - mass_center).transpose();
          }
          eigensolver.compute(scatter_matrix);
          eigenvalues = eigensolver.eigenvalues().real();
          eigenvectors = eigensolver.eigenvectors().real();
          int min_eigenvalue_index;
          double   lambda[2];
          double   min_eigenvalue = eigenvalues.minCoeff(&min_eigenvalue_index);
          size_t lambda_index = 0;
          for (size_t q = 0; q < 3; q++)
          {
            if (q == min_eigenvalue_index)
              continue;
            lambda[lambda_index] = eigenvalues(q)/min_eigenvalue;
            lambda_index ++;
          }
          double   min_lambda = std::min(lambda[0],lambda[1]);
          if (min_lambda > 4*sliding_window_size)
          {
            tmp.normal = eigenvectors.col(min_eigenvalue_index);
            if (tmp.normal.dot(mass_center) < 0)
              tmp.normal = -tmp.normal;
            tmp.mse = min_eigenvalue / valid_cnt;
            tmp.index = j * width_ + i;
            sliding_windows_.push_back(tmp);
          }
        }
      }
    }
    delete [] local_indices;
    //PCL_INFO("sliding windows finished: %d have been computed.\n", sliding_windows_.size());
  }

  template <typename PointT> void
  RGSegmentation<PointT>::investigate8Neighbors(const int index)
  {
    //pos_index - height -1     pos_index - 1  pos_index -1 + height
    // pos_index - height        pos_index      pos_index + height
    // pos_index + 1 - height    pos_index + 1  pos_index + height + 1

    int indices[8] =
    {
     index - width_ -1, index - 1, index + width_ - 1,
     index - width_, index + width_,
     index - width_ + 1, index + 1, index + width_ + 1
    };
    for (int i = 0; i < 8; i++)
    {
      if (!valid_[indices[i]] || visited_[indices[i]] || added_to_region_[indices[i]])
        continue;
      if ((points_[index] - points_[indices[i]]).squaredNorm () < max_neighbor_dis_)
      {
        neighbor_points_.push_back(indices[i]);
        visited_[indices[i]] = true;
      }
    }

  }

  template <typename PointT> void
  RGSegmentation<PointT>::investigate8Neighbors(const int x, const int y)
  {
    for (int i = -1; i <= 1; i++)
    {
      for (int j = -1; j <= 1; j++)
      {
        if ((i || j) && (x + i) >= 0 && (x + i) < width_)
        {
          if ((y + j) >= 0 && (y + j) < height_)
          {
            if (!valid_[(y + j) * width_ + x + i] ||
                visited_[(y + j) * width_ + x + i] ||
                added_to_region_[(y + j) * width_ + x +i])
              continue;
            //if (disPoint2Point(input_->at(x, y), input_->at(x + i, y + j)) < max_neighbor_dis_)
            if ((points_[y * width_ + x] - points_[(y + j) * width_ + x + i]).squaredNorm () < max_neighbor_dis_)
            {
              neighbor_points_.push_back((y + j) * width_ + x + i);
              visited_[(y + j) * width_ + x + i] = true;
            }
          }
          else if ((y + j) < 0)
          {
            if (!valid_[(height_ + y + j) * width_ + x + i] ||
                visited_[(height_ + y + j) * width_ + x + i] ||
                added_to_region_[(height_ + y + j) * width_ + x +i])
              continue;
            //if (disPoint2Point(input_->at(x, y), input_->at(x + i, height_ + y + j)) < max_neighbor_dis_)
            if ((points_[y * width_ + x] - points_[(y + j + height_) * width_ + x + i]).squaredNorm () < max_neighbor_dis_)
            {
              neighbor_points_.push_back((height_ + y + j) * width_ + x + i);
              visited_[(height_ + y + j) * width_ + x + i] = true;
            }
          }
          else if (y + j >= height_)
          {
            if (!valid_[(y + j - height_) * width_ + x + i] ||
                visited_[(y + j - height_) * width_ + x + i] ||
                added_to_region_[(y + j - height_) * width_ + x + i])
              continue;
            //if (disPoint2Point(input_->at(x, y), input_->at(x + i, y + j - height_)) < max_neighbor_dis_)
            if ((points_[y * width_ + x] - points_[(y + j - height_) * width_ + x + i]).squaredNorm () < max_neighbor_dis_)
            {
              neighbor_points_.push_back((y + j - height_) * width_ + x + i);
              visited_[(y + j - height_) * width_ + x + i] = true;
            }
          }
        }
      }
    }
  }

  template <typename PointT> void
  RGSegmentation<PointT>::investigate24Neighbors(const int x, const int y)
  {
 /*   for (int i = -2; i <= 2; i++)
    {
      for (int j = -2; j <= 2; j++)
      {
        if ((i || j) && (x + i) >= 0 && (x + i) < width_)
        {
          if ((y + j) >= 0 && (y + j) < static_cast<int>(height_))
          {
            if (!valid_[(y + j) * width_ + x + i] ||
                visited_[(y + j) * width_ + x + i] ||
                added_to_region_[(y + j) * width_ + x +i])
              continue;
            if (disPoint2Point(input_->at(x, y), input_->at(x + i, y + j)) < max_neighbor_dis_)
            {
              neighbor_points_.push_back(PointXY(x + i, y + j));
              visited_[(y + j) * width_ + x + i] = true;
            }
          }
          else if ((y + j) < 0)
          {
            if (!valid_[(height_ + y + j) * width_ + x + i] ||
                visited_[(height_ + y + j) * width_ + x + i] ||
                added_to_region_[(height_ + y + j) * width_ + x +i])
              continue;
            if (disPoint2Point(input_->at(x, y), input_->at(x + i, height_ + y + j)) < max_neighbor_dis_)
            {
              neighbor_points_.push_back(PointXY(x + i, height_ + y + j));
              visited_[(height_ + y + j) * width_ + x + i] = true;
            }
          }
          else if ((y + j) >= static_cast<int>(height_))
          {
            if (!valid_[(y + j - height_) * width_ + x + i] ||
                visited_[(y + j - height_) * width_ + x + i] ||
                added_to_region_[(y + j - height_) * width_ + x + i])
              continue;
            if (disPoint2Point(input_->at(x, y), input_->at(x + i, y + j - height_)) < max_neighbor_dis_)
            {
              neighbor_points_.push_back(PointXY(x + i, y + j - height_));
              visited_[(y + j - height_) * width_ + x + i] = 1;
            }
          }
        }
      }
    }*/
  }

  template <typename PointT> void
  RGSegmentation <PointT> :: applySegmentation (CloudXYZRGB::Ptr &output)
  {
    times.clear();
    planar_patches_.clear ();
    remained_points_.clear ();
    added_to_region_ = new bool [height_ * width_];
    memset(added_to_region_, false, height_ * width_);
    has_local_plane_ = new bool[height_ * width_];
    memset(has_local_plane_, false, height_ * width_);
    valid_ = new bool[height_ * width_];
    memset(valid_, false, height_ * width_);
    local_mse_ = new double[height_ * width_];
    memset(local_mse_, 0, height_ * width_);
    visited_ = new bool[height_ * width_];
    local_normals_.clear();
    local_normals_.resize(height_ * width_, Vector3d::Zero());

    EigenSolver<Matrix3d> eigensolver;
    Vector3d eigenvalues = Vector3d::Zero();
    Matrix3d eigenvectors = Matrix3d::Zero();
    Vector3d sum = Vector3d::Zero();
    Vector3d mass_center = Vector3d::Zero();
    Matrix3d scatter_matrix = Matrix3d::Zero();
    badpoints_num_ = 0;

    int valid_cnt = 0;
    for (int i = 0; i < points_.size (); i++)
    {
      if ((points_[i](0) != 0) || (points_[i](1) != 0) || (points_[i](2) != 0))
      {
        valid_[i] = true;
        valid_cnt ++;
      }
    }
    PCL_INFO("there are %d valid points in this point cloud.\n", valid_cnt);
    slidingWindow(sliding_window_size_);
    sort (sliding_windows_.begin(), sliding_windows_.end());
    for (size_t i = 0; i < sliding_windows_.size(); i++)
    {
      has_local_plane_[sliding_windows_[i].index] = true;
      local_mse_[sliding_windows_[i].index] = sliding_windows_[i].mse;
      local_normals_[sliding_windows_[i].index] = sliding_windows_[i].normal;
    }
    size_t index = 0;
    while (index < sliding_windows_.size())
    {
      while (index < sliding_windows_.size() &&
             (added_to_region_[sliding_windows_[index].index] || !valid_[sliding_windows_[index].index]))
      {
        index ++;
      }
      if (index == static_cast<int>(sliding_windows_.size()) || sliding_windows_[index].mse > max_seed_mse_)
        break;
      memset(visited_, false, height_ * width_);
      bool isSmall = true;
      PlanarSegment tmp_pp;
      neighbor_points_.clear();
      neighbor_points_.push_back(sliding_windows_[index].index);
      visited_[sliding_windows_[index].index] = true;
      added_to_region_[sliding_windows_[index].index] = true;
      while (!neighbor_points_.empty())
      {
        int pos_index = neighbor_points_.front();
        neighbor_points_.erase(neighbor_points_.begin());
        Vector3d point3d = points_[pos_index];
        if (tmp_pp.point_num < 7)
        {
          tmp_pp.points.push_back(pos_index);
          tmp_pp.point_num ++;
          tmp_pp.sum += point3d;
          added_to_region_[pos_index] = true;
          investigate8Neighbors(pos_index % width_, pos_index / width_);
          isSmall = true;
        }
        else if (tmp_pp.point_num >= 7 && isSmall == true)
        {
          tmp_pp.mass_center = tmp_pp.sum / 7.0;
          for (vector<int>::iterator it = tmp_pp.points.begin(); it != tmp_pp.points.end(); it++)
            tmp_pp.scatter_matrix += (points_[*it] - tmp_pp.mass_center) * (points_[*it] - tmp_pp.mass_center).transpose();
          eigensolver.compute(tmp_pp.scatter_matrix);
          eigenvalues = eigensolver.eigenvalues().real();
          eigenvectors = eigensolver.eigenvectors().real();
          int min_eigenvalue_index = 100;
          double min_eigenvalue = eigenvalues.minCoeff(&min_eigenvalue_index);
          tmp_pp.normal = eigenvectors.col(min_eigenvalue_index);
          tmp_pp.bias = tmp_pp.normal.dot(tmp_pp.mass_center);
          tmp_pp.mse = min_eigenvalue / 7.0;
          if (tmp_pp.bias < 0)
          {
            tmp_pp.bias = -tmp_pp.bias;
            tmp_pp.normal = -tmp_pp.normal;
          }
          isSmall = false;
        }
        else if (tmp_pp.point_num >= 7 && isSmall == false)
        {
          sum = tmp_pp.sum + point3d;
          mass_center = sum / static_cast<double>(tmp_pp.point_num + 1);
          scatter_matrix = tmp_pp.scatter_matrix + point3d * point3d.transpose() - sum * mass_center.transpose() + tmp_pp.sum * tmp_pp.mass_center.transpose();
          eigensolver.compute(scatter_matrix);
          eigenvalues = eigensolver.eigenvalues().real();
          eigenvectors = eigensolver.eigenvectors().real();
          int min_eigenvalue_index;
          double min_eigenvalue = eigenvalues.minCoeff(&min_eigenvalue_index);

          if (min_eigenvalue / static_cast<double>(tmp_pp.point_num + 1) > max_segment_mse_)
          {
            visited_[pos_index] = false;
            continue;
          }
          if (fabs(eigenvectors.col(min_eigenvalue_index).dot(mass_center - point3d)) > max_point2plane_dis_)
          {
            visited_[pos_index] = false;
            continue;
          }
          if (has_local_plane_[pos_index] && local_mse_[pos_index] < max_local_mse_)
          {
            double   dot_product = local_normals_[pos_index].dot(eigenvectors.col(min_eigenvalue_index));
            if ( fabs(dot_product) < max_angle_difference_)
            {
              visited_[pos_index] = false;
              continue;
            }
          }
          tmp_pp.point_num ++;
          tmp_pp.points.push_back(pos_index);
          added_to_region_[pos_index] = true;
          tmp_pp.scatter_matrix = scatter_matrix;
          tmp_pp.sum = sum;
          tmp_pp.mass_center = mass_center;
          tmp_pp.normal = eigenvectors.col(min_eigenvalue_index);
          tmp_pp.bias = tmp_pp.normal.dot(mass_center);
          if (tmp_pp.bias < 0)
          {
            tmp_pp.bias = -tmp_pp.bias;
            tmp_pp.normal = -tmp_pp.normal;
          }
          if (nearest_neighbor_size_ == 8)
          {
            int u = pos_index % width_, v = pos_index / width_;
            if (u > 0 && u < width_ - 1 && v > 0 && v < height_ - 1)
            {
              investigate8Neighbors(pos_index);
            }
            else
            {
              investigate8Neighbors(u, v);
            }
          }
          if (nearest_neighbor_size_ == 24)
          {
            investigate24Neighbors(pos_index % width_, pos_index / width_);
          }
        }
      }//end while (!neighbor_points_.empty())
      if (tmp_pp.point_num > min_segment_size_)
      {
        planar_patches_.push_back(tmp_pp);
        gettimeofday (&tmp_time, NULL);
        times.push_back (tmp_time);
      }
      else
      {
        badpoints_num_ += tmp_pp.point_num;
        remained_points_.insert(remained_points_.begin(), tmp_pp.points.begin(), tmp_pp.points.end());
      }
    }//endof while (index < static_cast<int> (sliding_windows_.size()))
    PCL_INFO ("%d segments have been identified.\n", planar_patches_.size());
    PCL_INFO ("%d points have not been identified to any segment.\n", badpoints_num_);
    delete[] added_to_region_;
    delete[] has_local_plane_;
    delete[] valid_;
    delete[] local_mse_;
    delete[] visited_;
    //colorEncoding(output);
  }

  template <typename PointT> void
  RGSegmentation <PointT> :: parameterUncertainty()
  {
     double   noisysigma = 0.02; //3.3e-5;
     Matrix4d H;
     Matrix4d C;
     double   Hdd;
     Vector3d Hnd;
     Matrix3d Hnn;
     double   omega;
     double   omega_sum = 0;
     Vector3d weighted_sum = Vector3d::Zero();
     Vector3d Pc;
     Vector3d optimal_normal;
     double   optimal_bias;
     Matrix3d eye = Matrix3d::Identity();
     Matrix3d M = Matrix3d::Zero();
     for (size_t i = 0; i < planar_patches_.size (); i++)
     {
       for (size_t j = 0; j < planar_patches_[i].point_num; j++)
       {
         Vector3d point = points_[planar_patches_[i].points[j]];
         //omega   = 1/(point.dot(point)*point.dot(point)*noisysigma*noisysigma);
         omega   = 1/(noisysigma*noisysigma);
         omega_sum       += omega;
         weighted_sum    += omega*point;
       }
       Pc = weighted_sum/omega_sum;
       for (size_t j = 0; j < planar_patches_[i].point_num; j++)
       {
         Vector3d point = points_[planar_patches_[i].points[j]];
         //omega   = 1/(point.dot(point)*point.dot(point)*noisysigma*noisysigma);
         omega   = 1/(noisysigma*noisysigma);
         M       += omega*(point-Pc)*(point-Pc).transpose();
       }
       EigenSolver<Matrix3d> eigen_solver(M);
       Vector3d eigen_values  = eigen_solver.eigenvalues().real();
       Matrix3d eigen_vectors = eigen_solver.eigenvectors().real();
       double   min_eigen_value = eigen_values.minCoeff();
       for(int k = 0; k < 3; k++)
       {
         if(eigen_values(k) == min_eigen_value)
         {
           optimal_normal = eigen_vectors.col(k);
         }
       }
       optimal_bias = optimal_normal.dot(Pc);
       if(optimal_bias<0)
       {
         optimal_normal = -optimal_normal;
         optimal_bias   = -optimal_bias;
       }
       Hdd = -omega_sum;
       Hnd = -Hdd*Pc;
       double   tmp = optimal_normal.transpose()*M*optimal_normal;
       Hnn = -M + Hdd*Pc*Pc.transpose() + tmp*eye;

       H.block<3,3>(0,0) = Hnn;
       H.block<3,1>(0,3) = Hnd;
       H.block<1,3>(3,0) = Hnd.transpose();
       H(3,3) = Hdd;
       EigenSolver<Matrix4d> Matrix4d_eigen_solver(-H);
       Vector4d tmp_vector4d = Matrix4d_eigen_solver.eigenvalues().real();
       //cout<<tmp_vector4d(0)<<", "<<tmp_vector4d(1)<<", "<<tmp_vector4d(2)<<", "<<tmp_vector4d(3)<<endl;
       double   min = tmp_vector4d.minCoeff();
       for(int k = 0; k < 4; k++)
       {
         if(tmp_vector4d(k) == min)
         {
           tmp_vector4d(k) = 1;
           //cout<<k<<" ";
         }
         else
         {
           //tmp_vector4d(i) = 1/tmp_vector4d(i);
         }
       }
       //cout<<endl;
       //planar_patches_[i].uncertainty = tmp_vector4d(0)*tmp_vector4d(1)*tmp_vector4d(2)*tmp_vector4d(3);
       //cout<<planar_patches_[i].uncertainty<<endl;
     }
  }

  template <typename PointT> void
  RGSegmentation <PointT> :: colorEncoding (CloudXYZRGB::Ptr &output)
  {
    std::vector<RGB> colors;
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

    output->points.clear ();
    output->points.resize(width_ * height_);
    srand ( time(NULL) );
    int cnt = 0;
    for (size_t i = 0; i < planar_patches_.size(); i++)
    {
      if (planar_patches_[i].area < 0.2)
        continue;
      int gray = 255;
      int color_index;
      do{
        color_index = rand() % colors.size();
        gray = (colors[color_index].r + colors[color_index].g + colors[color_index].b) / 3;
      }
      while (gray < 75 || gray > 150);
      for (vector<int>::iterator it = planar_patches_[i].points.begin(); it != planar_patches_[i].points.end (); it++)
      {
        output->points[cnt].x = points_[*it](0);//input_->points[*it].x;
        output->points[cnt].y = points_[*it](1);//input_->points[*it].y;
        output->points[cnt].z = points_[*it](2);//input_->points[*it].z;
        output->points[cnt].r = colors[color_index].r;
        output->points[cnt].g = colors[color_index].g;
        output->points[cnt].b = colors[color_index].b;
        cnt ++;
      }
    }
    output->points.erase(output->points.begin() + cnt, output->points.end());
    output->height = 1;
    output->width = cnt;
    output->resize(cnt);
    std::cout << "size of output: " << output->size() << std::endl;
  }
}
#endif
