/***
 * @Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-27 11:32:06
 * @LastEditors: Tianyun Xuan
 */
#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <string>
#include <vector>
namespace Evaluation {
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr CloudPtr;

class Evaluation {
 private:
  int m_iteration_;
  double m_ground_threshold_;
  double m_ground_tolerence_;

 public:
  Evaluation(/* args */);
  ~Evaluation();
  void compute(const CloudPtr& first_in, const CloudPtr& second_in,
               CloudPtr& first_out, CloudPtr& second_out);
  void obb_overlap(const CloudPtr& first_in, const CloudPtr& second_in,
                     CloudPtr& first_out, CloudPtr& second_out);
  void tree_overlap(const CloudPtr& first_in, const CloudPtr& second_in,
                     CloudPtr& first_out, CloudPtr& second_out);
  void ground_removal(const CloudPtr& first_in, const CloudPtr& second_in,
                      CloudPtr& first_out, CloudPtr& second_out);
  void set_ground_param(const int& iteration, const double& thresholh,
                        const double& tolerence) {
    m_iteration_ = iteration;
    m_ground_threshold_ = thresholh;
    m_ground_tolerence_ = tolerence;
  }
};

}  // namespace Evaluation