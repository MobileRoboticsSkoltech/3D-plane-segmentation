#include "subwindow_region_growing/subwindow_region_growing.h"
#include <algorithm>
using namespace tams;
void
SubwindowRGSegmentation::applySegmentation()
{
  width_ = cloud_->width;
  height_ = cloud_->height;
  size_ = height_ * width_;
  valid_ = new bool [size_];
  memset (valid_, false, size_ * sizeof (bool));
  valid_points_ = 0;
  for (int i = 0; i < size_; i++)
  {
    if (points_[i](0) != 0.0 || points_[i](1) != 0.0 || points_[i](2) != 0.0)
    {
      valid_[i] = true;
      valid_points_ ++;
    }
  }
  subwindows(parameters_.subwindow_side_length);

  int subwindow_size = parameters_.subwindow_side_length * parameters_.subwindow_side_length;
  planar_patches_.clear();
  remained_points_.clear();
  added_to_region_ = new bool [subwindows_.size()];
  memset (added_to_region_, false, subwindows_.size() * sizeof (bool));
  visited_ = new bool [subwindows_.size()];

  Vector3d sum = Vector3d::Zero();
  Vector3d mass_center = Vector3d::Zero();
  Vector3d normal = Vector3d::Zero();
  Matrix3d second_moment = Matrix3d::Zero();
  Matrix3d scatter_matrix = Matrix3d::Zero();
  int point_num = 0;
  double bias;

  EigenSolver<Matrix3d> eigensolver;
  Vector3d eigenvalues = Vector3d::Zero();
  Matrix3d eigenvectors = Matrix3d::Zero();

  int index = 0;
  int pos = 0;
  while (index < planar_subwindows_cnt_)
  {
    while (index < planar_subwindows_cnt_ && added_to_region_[sorted_subwindows_[index].index])
    {
      index ++;
    }
    if (index == planar_subwindows_cnt_ && subwindows_[sorted_subwindows_[index].index].mse >= parameters_.max_segment_mse * 0.1)
      break;
    memset (visited_, false, subwindows_.size() * sizeof (bool));
    PlanarSegment tmp_pp;
    neighbors_.clear();
    pos = sorted_subwindows_[index].index;
    tmp_pp.sum = subwindows_[pos].sum;
    tmp_pp.mass_center = subwindows_[pos].mass_center;
    tmp_pp.normal = subwindows_[pos].normal;
    tmp_pp.second_moment = subwindows_[pos].second_moment;
    tmp_pp.bias = subwindows_[pos].bias;
    tmp_pp.mse = subwindows_[pos].mse;
    tmp_pp.point_num = subwindows_[pos].point_num;
    tmp_pp.points.resize(valid_points_);
    for (int i = 0, *p = valid_indices_ + pos * subwindow_size; i < tmp_pp.point_num; i++, p++)
    {
      tmp_pp.points[i] = *p;
    }
    investigate8Neighbors(pos);
    int cnt = 0;
    for (int i = 0; i < neighbors_.size(); i++)
    {
      if (isPlanar_[neighbors_[i]] && subwindows_[neighbors_[i]].normal.dot(tmp_pp.normal) > parameters_.min_dot_product)
        cnt ++;
    }
    if (cnt < 5)
    {
      index ++;
      continue;
    }
    while (!neighbors_.empty())
    {
      pos = neighbors_.front();
      neighbors_.erase(neighbors_.begin());
      if (subwindows_[pos].normal.dot(tmp_pp.normal) < parameters_.min_dot_product)
      {
        visited_[pos] = false;
        continue;
      }
      if (fabs(tmp_pp.normal.dot(subwindows_[pos].mass_center) - tmp_pp.bias) > parameters_.max_mass2plane_dis)
      {
        visited_[pos] = false;
        continue;
      }
      point_num = tmp_pp.point_num + subwindows_[pos].point_num;
      sum = tmp_pp.sum + subwindows_[pos].sum;
      mass_center = sum / static_cast<double >(point_num);
      second_moment = tmp_pp.second_moment + subwindows_[pos].second_moment;
      scatter_matrix = second_moment - sum * mass_center.transpose();
      eigensolver.compute(scatter_matrix);
      eigenvalues = eigensolver.eigenvalues().real();
      eigenvectors = eigensolver.eigenvectors().real();
      int min_eigenvalue_index;
      double min_eigenvalue = eigenvalues.minCoeff(&min_eigenvalue_index);
      if (min_eigenvalue/static_cast<double >(point_num) > parameters_.max_segment_mse)
      {
        visited_[pos] = false;
        continue;
      }
      normal = eigenvectors.col(min_eigenvalue_index);
      bias = normal.dot(mass_center);
      if (bias < 0)
      {
        normal = -normal;
        bias = -bias;
      }
      tmp_pp.sum = sum;
      tmp_pp.mass_center = mass_center;
      tmp_pp.second_moment = second_moment;
      tmp_pp.normal = normal;
      tmp_pp.bias = bias;
      for (int i = tmp_pp.point_num, *p = valid_indices_ + pos * subwindow_size; i < point_num; i++, p++)
      {
        tmp_pp.points[i] = *p;
      }
      tmp_pp.point_num = point_num;
      added_to_region_[pos] = true;
      investigate8Neighbors(pos);
    }
    tmp_pp.points.erase(tmp_pp.points.begin() + tmp_pp.point_num, tmp_pp.points.end());
    if (tmp_pp.point_num > parameters_.min_segment_size)
    {
      planar_patches_.push_back(tmp_pp);
    }
    else
    {
      //badpoints_num_ += tmp_pp.point_num;
      //remained_points_.insert(remained_points_.begin(), tmp_pp.points.begin(), tmp_pp.points.end());
    }
    index ++;
  }

  delete [] valid_;
  delete [] added_to_region_;
  delete [] visited_;
  delete [] isPlanar_;
  delete [] valid_indices_;
  //PCL_INFO ("%d segments have been detected.\n", planar_patches_.size());
}

void
SubwindowRGSegmentation :: preprocessing ()
{
  points_.clear();
  points_.resize(cloud_->size(), Vector3d::Zero());
  double threshold = 30 * 30;
  for (int i = 0; i < cloud_->size(); i++)
  {
    if (cloud_->points[i].z < 0 &&
       (cloud_->points[i].x * cloud_->points[i].x +
        cloud_->points[i].y * cloud_->points[i].y) < 0.7225)
      continue;
    if (cloud_->points[i].x * cloud_->points[i].x +
        cloud_->points[i].y * cloud_->points[i].y +
        cloud_->points[i].z * cloud_->points[i].z > threshold)
      continue;
    if (cloud_->points[i].x != 0.0 || cloud_->points[i].y != 0.0 || cloud_->points[i].z != 0.0)
    {
      points_[i] = Vector3d(cloud_->points[i].x, cloud_->points[i].y, cloud_->points[i].z);
    }
  }
}

void
SubwindowRGSegmentation :: randomColours (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output, bool project2plane)
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

  output->points.clear();
  output->points.resize(cloud_->size());

  srand ( time(NULL) );
  int cnt = 0;
  for (PlanarSegment::StdVector::iterator it = planar_patches_.begin(); it != planar_patches_.end(); it++)
  {
    int gray = 0;
    int color_index;
    while (gray<75)
    {
      color_index = rand() % colors.size();
      gray = (colors[color_index].r + colors[color_index].g + colors[color_index].b) / 3;
    }
    int32_t rgb_integer = (colors[color_index].r << 16) | (colors[color_index].g << 8) | colors[color_index].b;
    double rgb = *(double *)(&rgb_integer);
    if (!project2plane)
    {
      for (int j = 0; j < it->points.size(); j++)
      {
        int index = it->points[j];
        output->points[cnt].x = cloud_->points[index].x;
        output->points[cnt].y = cloud_->points[index].y;
        output->points[cnt].z = cloud_->points[index].z;
        output->points[cnt].rgb = rgb;
        cnt ++;
      }
    }
    else
    {
      for (int j = 0; j < it->points.size(); j++)
      {
        int index = it->points[j];
        double dis = cloud_->points[index].x * it->normal(0) +
                    cloud_->points[index].y * it->normal(1) +
                    cloud_->points[index].z * it->normal(2) - it->bias;
        output->points[cnt].x = cloud_->points[index].x - dis * it->normal(0);
        output->points[cnt].y = cloud_->points[index].y - dis * it->normal(1);
        output->points[cnt].z = cloud_->points[index].z - dis * it->normal(2);
        output->points[cnt].rgb = rgb;
        cnt ++;
      }
    }
  }
  output->points.erase(output->points.begin() + cnt, output->points.end());
  output->height = 1;
  output->width = cnt;
  output->resize(cnt);
}

void
SubwindowRGSegmentation::subwindows(int side_length)
{
//  struct timeval tpstart,tpend;
//  gettimeofday(&tpstart,NULL);
  subwindows_.clear ();
  subwindows_height_ = static_cast<int>(cloud_->height/side_length);
  subwindows_width_ = static_cast<int>(cloud_->width/side_length);
  subwindows_.resize(subwindows_height_ * subwindows_width_);
  isPlanar_ = new bool [subwindows_.size()];
  memset (isPlanar_, false, subwindows_.size() * sizeof (bool));
  Vector3d sum = Vector3d::Zero(), mass_center = Vector3d::Zero(), normal = Vector3d::Zero();
  Matrix3d scatter_matrix = Matrix3d::Zero(), second_moment = Matrix3d::Zero();
  double bias = 0;
  EigenSolver<Matrix3d> eigensolver;
  Vector3d eigenvalues = Vector3d::Zero();
  Matrix3d eigenvectors = Matrix3d::Zero();
  int valid_cnt = 0;
  int row = 0, col = 0;
  int subwindow_size = side_length * side_length;
  valid_indices_ = new int [subwindows_.size() * subwindow_size];
  int pos = 0;
  int planar_cnt = 0;
  std::vector<Subwindow>::iterator it = subwindows_.begin();
  int subwindow_index = 0;
  int *pbegin = NULL;
  int *pend = NULL;
  int *p = NULL;
  for (int i = 0; i < height_ - side_length; i += side_length)
  {
    for (int j = 0; j < width_ - side_length; j += side_length)
    {
      pbegin = valid_indices_ + subwindow_index * subwindow_size;
      sum = Vector3d::Zero();
      scatter_matrix = Matrix3d::Zero();
      second_moment = Matrix3d::Zero();
      valid_cnt = 0;
      p = pbegin;
      for (row = i; row < i + side_length; row++)
      {
        for (col = j; col < j + side_length; col++)
        {
          pos = row * width_ + col;
          if (!valid_[pos])
            continue;
          valid_cnt++;
          *(p++) = pos;
        }
      }
      pend = p;
      it->point_num = valid_cnt;
      if (valid_cnt < static_cast<int>(subwindow_size * 0.65))
      {
        it++;
        subwindow_index ++;
        continue;
      }
      for (p = pbegin; p < pend; p++)
      {
        sum += points_[*p];
        second_moment += points_[*p] * points_[*p].transpose();
      }
      mass_center = sum / static_cast<double >(valid_cnt);
      for (p = pbegin; p < pend; p++)
      {
        scatter_matrix += (points_[*p] - mass_center) * (points_[*p] - mass_center).transpose();
      }
      eigensolver.compute(scatter_matrix);
      eigenvalues = eigensolver.eigenvalues().real();
      eigenvectors = eigensolver.eigenvectors().real();
      int min_eigenvalue_index;
      double lambda[2];
      double min_eigenvalue = eigenvalues.minCoeff(&min_eigenvalue_index);
      int lambda_index = 0;
      for (size_t q = 0; q < 3; q++)
      {
        if (q == min_eigenvalue_index)
          continue;
        lambda[lambda_index] = eigenvalues(q) / min_eigenvalue;
        lambda_index ++;
      }
      double min_lambda = std::min(lambda[0],lambda[1]);
      if (min_lambda < 2.5)
      {
        it++;
        subwindow_index ++;
        continue;
      }
      it->normal = eigenvectors.col(min_eigenvalue_index);
      it->bias = it->normal.dot(mass_center);
      if (it->bias < 0)
      {
        it->normal = -it->normal;
        it->bias = -it->bias;
      }
      it->sum = sum;
      it->mass_center = mass_center;
      it->second_moment = second_moment;
      it->mse = min_eigenvalue / static_cast<double >(valid_cnt);
      isPlanar_[subwindow_index] = true;
      planar_cnt ++;
      it++;
      subwindow_index ++;
    }
  }
  //PCL_INFO("Computing subwindows finished: %d have been computed, in which there are total %d planar subwindows.\n",
  //         subwindows_.size(), planar_cnt);
  planar_subwindows_cnt_ = planar_cnt;
  sorted_subwindows_.resize(planar_subwindows_cnt_);
  subwindow_index = 0;
  for (int i = 0; i < subwindows_.size(); i++)
  {
    if (isPlanar_[i])
    {
      sorted_subwindows_[subwindow_index].index = i;
      sorted_subwindows_[subwindow_index].mse = subwindows_[i].mse;
      subwindow_index++;
    }
  }
  std::sort (sorted_subwindows_.begin(), sorted_subwindows_.end());
}

void
SubwindowRGSegmentation::investigate8Neighbors (const int index)
{
  int row = index / subwindows_width_;
  int col = index % subwindows_width_;
  for (int i = -1; i < 2; i++)
  {
    for (int j = -1; j < 2; j++)
    {
      if ((i || j) && row + i >= 0 && row + i < subwindows_height_ && col + j >= 0 && col + j < subwindows_width_)
      {
        int pos = (row + i) * subwindows_width_ + col + j;
        if (!visited_[pos] && !added_to_region_[pos] && isPlanar_[pos])
        {
          neighbors_.push_back (pos);
          visited_[pos] = true;
        }
      }
    }
  }
}
