/***
 * @Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-27 11:34:08
 * @LastEditors: Tianyun Xuan
 */

#include "evaluation.h"

#include <pcl/octree/octree.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace Evaluation {
Evaluation::Evaluation(/* args */) {
  m_iteration_ = 10000;
  m_ground_threshold_ = 0.1;
  m_ground_tolerence_ = 0.1;
}
Evaluation::~Evaluation() {}

void Evaluation::compute(const CloudPtr& first_in, const CloudPtr& second_in,
                         CloudPtr& first_out, CloudPtr& second_out) {
  CloudPtr first_temp = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  CloudPtr second_temp = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  CloudPtr first_temp2 = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  CloudPtr second_temp2 = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  // obb_overlap(first_in, second_in, first_out, second_out);
  obb_overlap(first_in, second_in, first_temp, second_temp);
  ground_removal(first_temp, second_temp, first_temp2, second_temp2);
  tree_overlap(first_temp2, second_temp2, first_out, second_out);
}

void Evaluation::tree_overlap(const CloudPtr& first_in,
                              const CloudPtr& second_in, CloudPtr& first_out,
                              CloudPtr& second_out) {
  double radius = 0.3;
  first_out->clear();
  second_out->clear();
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree(radius);
  octree.setInputCloud(first_in);
  octree.addPointsFromInputCloud();
  std::set<int> ids{};

  for (auto& point : second_in->points) {
    std::vector<int> indices;
    octree.voxelSearch(point, indices);
    if (indices.size()) {
      second_out->emplace_back(point);
      for (auto& id : indices) {
        ids.insert(id);
      }
    }
  }

  for (auto& id : ids) {
    first_out->emplace_back(first_in->points[id]);
  }
}

void Evaluation::obb_overlap(const CloudPtr& first_in,
                             const CloudPtr& second_in, CloudPtr& first_out,
                             CloudPtr& second_out) {
  double f_y_max = std::numeric_limits<double>::min();
  double f_z_max = std::numeric_limits<double>::min();
  double s_y_max = std::numeric_limits<double>::min();
  double s_z_max = std::numeric_limits<double>::min();
  double f_y_min = std::numeric_limits<double>::max();
  double f_z_min = std::numeric_limits<double>::max();
  double s_y_min = std::numeric_limits<double>::max();
  double s_z_min = std::numeric_limits<double>::max();

  for (auto& point : first_in->points) {
    f_y_max = std::fmax(point.y, f_y_max);
    f_y_min = std::fmin(point.y, f_y_min);
    f_z_max = std::fmax(point.z, f_z_max);
    f_z_min = std::fmin(point.z, f_z_min);
  }

  for (auto& point : second_in->points) {
    s_y_max = std::fmax(point.y, s_y_max);
    s_y_min = std::fmin(point.y, s_y_min);
    s_z_max = std::fmax(point.z, s_z_max);
    s_z_min = std::fmin(point.z, s_z_min);
  }

  double y_max = std::fmin(f_y_max, s_y_max);
  double y_min = std::fmax(f_y_min, s_y_min);
  double z_max = std::fmin(f_z_max, s_z_max);
  double z_min = std::fmax(f_z_min, s_z_min);

  for (auto& point : first_in->points) {
    if (point.y < y_max && point.y > y_min && point.z < z_max &&
        point.z > z_min) {
      first_out->emplace_back(point);
    }
  }

  for (auto& point : second_in->points) {
    if (point.y < y_max && point.y > y_min && point.z < z_max &&
        point.z > z_min) {
      second_out->emplace_back(point);
    }
  }
}

void Evaluation::ground_removal(const CloudPtr& first_in,
                                const CloudPtr& second_in, CloudPtr& first_out,
                                CloudPtr& second_out) {
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  CloudPtr source = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::concatenate(*first_in, *second_in, *source);

  pcl::SACSegmentation<pcl::PointXYZI> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(m_iteration_);
  seg.setDistanceThreshold(m_ground_threshold_);
  seg.setInputCloud(source);
  seg.segment(*inliers_plane, *coefficients_plane);

  // seg.setInputCloud(first_in);
  // seg.segment(*inliers_plane, *coefficients_plane);
  // pcl::copyPointCloud(*first_in, *inliers_plane, *first_out);

  // seg.setInputCloud(second_in);
  // seg.segment(*inliers_plane, *coefficients_plane);
  // pcl::copyPointCloud(*second_in, *inliers_plane, *second_out);

  double a = coefficients_plane->values[0];
  double b = coefficients_plane->values[1];
  double c = coefficients_plane->values[2];
  double d = coefficients_plane->values[3];

  first_out->clear();
  second_out->clear();

  for (auto& point : first_in->points) {
    if (fabs(a * point.x + b * point.y + c * point.z + d) >
        m_ground_tolerence_) {
      first_out->emplace_back(point);
    }
  }

  for (auto& point : second_in->points) {
    if (fabs(a * point.x + b * point.y + c * point.z + d) >
        m_ground_tolerence_) {
      second_out->emplace_back(point);
    }
  }
}
}  // namespace Evaluation
