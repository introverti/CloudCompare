#include "calibration.h"

#include "logger.h"
namespace Calibration {
void Operator::save_pcd(std::string name, const CloudPtr &cloud) {
  if (save_flag_ && cloud->size()) {
    pcl::io::savePCDFileBinary(name, *cloud);
  }
}

Eigen::Matrix3f transforme_normal_normal_3d(const Eigen::Vector3f &start,
                                            const Eigen::Vector3f &final) {
  Eigen::Vector3f start_norm = start.normalized();
  Eigen::Vector3f final_norm = final.normalized();
  Eigen::Vector3f rotation_axis = final_norm.cross(start_norm);

  float rcos = start_norm.dot(final_norm);
  float rsin = rotation_axis.norm();
  float u = rotation_axis.x();
  float v = rotation_axis.y();
  float w = rotation_axis.z();
  Eigen::Matrix3f rotation;
  rotation << rcos + u * u * (1 - rcos), w * rsin + v * u * (1 - rcos),
      -v * rsin + w * u * (1 - rcos), -w * rsin + u * v * (1 - rcos),
      rcos + v * v * (1 - rcos), u * rsin + w * v * (1 - rcos),
      v * rsin + u * w * (1 - rcos), -u * rsin + v * w * (1 - rcos),
      rcos + w * w * (1 - rcos);
  return rotation;
}

void trans_plane_plane_3d(const std::vector<float> plane_coeffs,
                          const std::vector<float> new_plane_coeffs,
                          std::vector<float> &rotate,
                          std::vector<float> &trans) {
  float cross_vector[3];

  float x1 = new_plane_coeffs[0];  // 0
  float y1 = new_plane_coeffs[1];  // 0
  float z1 = new_plane_coeffs[2];  // 1
  // float d1 = new_plane_coeffs[3];  // 0

  float x2 = plane_coeffs[0];  // a
  float y2 = plane_coeffs[1];  // b
  float z2 = plane_coeffs[2];  // c
  float d2 = plane_coeffs[3];  // d

  // rotate theta from plane_coeffs to new_plane_coeffs
  float theta = acosf(x1 * x2 + y1 * y2 + z1 * z2);
  cross_vector[0] = (y2 * z1 - y1 * z2);  // 0
  cross_vector[1] = (z2 * x1 - z1 * x2);  // 0
  cross_vector[2] = (x2 * y1 - x1 * y2);  // 1
  float norm = 1.f / sqrt(cross_vector[0] * cross_vector[0] +
                          cross_vector[1] * cross_vector[1] +
                          cross_vector[2] * cross_vector[2]);
  float u = cross_vector[0] * norm;
  float v = cross_vector[1] * norm;
  float w = cross_vector[2] * norm;
  float uu = u * u;
  float ww = w * w;
  float vv = v * v;
  float uv = u * v;
  float uw = u * w;
  float vw = v * w;
  float cosa = cosf(theta);
  float sina = sinf(theta);
  // cross line's projected point in plane y0z
  float a0 = 0;
  float b0 = 0;
  float c0 = -d2 / z2;

  // move to principle point and use rodrigues transform
  // and move to the projected point again
  rotate[0] = uu + (vv + ww) * cosa;
  rotate[1] = uv * (1 - cosa) - w * sina;
  rotate[2] = uw * (1 - cosa) + v * sina;
  trans[0] = (a0 * (vv + ww) - u * (b0 * v + c0 * w)) * (1 - cosa) +
             (b0 * w - c0 * v) * sina;

  rotate[3] = uv * (1 - cosa) + w * sina;
  rotate[4] = vv + (uu + ww) * cosa;
  rotate[5] = vw * (1 - cosa) - u * sina;
  trans[1] = (b0 * (uu + ww) - v * (a0 * u + c0 * w)) * (1 - cosa) +
             (c0 * u - a0 * w) * sina;

  rotate[6] = uw * (1 - cosa) - v * sina;
  rotate[7] = vw * (1 - cosa) + u * sina;
  rotate[8] = ww + (uu + vv) * cosa;
  trans[2] = (c0 * (uu + vv) - w * (a0 * u + b0 * v)) * (1 - cosa) +
             (a0 * v - b0 * u) * sina;
}

void Operator::findAround(CloudPtr source, CloudPtr &around,
                          CloudPtr &attention, const float &intensity) {
  around->clear();
  attention->clear();
  for (auto &point : source->points) {
    if (point.intensity >= intensity) {
      attention->emplace_back(point);
    } else {
      around->emplace_back(point);
    }
  }
}

void Operator::guide_filter(CloudPtr source, CloudPtr &result) {
  result->clear();
  unsigned int k = 20;
  double epsilon = 0.05;
  kdtree_->setInputCloud(source);
  inno_log_info("Guide filter Input size : %ld", source->size());
  for (size_t i = 0; i < source->size(); ++i) {
    std::vector<int> indices(0, 0);
    indices.reserve(k);
    std::vector<float> dist(0, 0.0);
    CloudPtr neigh_points = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
    dist.reserve(k);
    if (kdtree_->nearestKSearch(source->points[i], k, indices, dist) > 0) {
      pcl::copyPointCloud(*source, indices, *neigh_points);
      pcl::PointXYZ point_mean(0.0, 0.0, 0.0);
      double neigh_mean_2 = 0.0;
      for (auto neigh_point : neigh_points->points) {
        point_mean.x += neigh_point.x;
        point_mean.y += neigh_point.y;
        point_mean.z += neigh_point.z;
        neigh_mean_2 += ((neigh_point.x) * double(neigh_point.x)) +
                        (double(neigh_point.y) * double(neigh_point.y)) +
                        (double(neigh_point.z) * double(neigh_point.z));
      }
      point_mean.x /= neigh_points->size();
      point_mean.y /= neigh_points->size();
      point_mean.z /= neigh_points->size();
      neigh_mean_2 /= neigh_points->size();

      double point_mean_2 = (point_mean.x * point_mean.x) +
                            (point_mean.y * point_mean.y) +
                            (point_mean.z * point_mean.z);
      double a = (neigh_mean_2 - point_mean_2) /
                 (neigh_mean_2 - point_mean_2 + epsilon);
      pcl::PointXYZ b;
      b.x = (1.0 - a) * point_mean.x;
      b.y = (1.0 - a) * point_mean.y;
      b.z = (1.0 - a) * point_mean.z;

      pcl::PointXYZI smoothed_point;
      smoothed_point.x = a * source->points[i].x + b.x;
      smoothed_point.y = a * source->points[i].y + b.y;
      smoothed_point.z = a * source->points[i].z + b.z;
      smoothed_point.intensity = source->points[i].intensity;
      result->push_back(smoothed_point);
    }
  }
  result->width = result->size();
  result->height = 1;
  result->resize(double(result->width) * double(result->height));
  inno_log_info("Guide filter Output size : %ld", result->size());
}

void Operator::outline_remove(CloudPtr source, CloudPtr &result) {
  result->clear();
  pcl::RadiusOutlierRemoval<pcl::PointXYZI> rout;
  rout.setInputCloud(source);
  rout.setRadiusSearch(0.05);
  rout.setMinNeighborsInRadius(5);
  rout.filter(*result);
}
void Operator::dbscan(CloudPtr source, CloudPtr &result) {
  result->clear();
#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2
  std::vector<int> nn_indices;
  std::vector<float> nn_distances;
  std::vector<bool> is_noise(source->points.size(), false);
  std::vector<int> types(source->points.size(), UN_PROCESSED);
  kdtree_->setInputCloud(source);
  double eps = 0.03;
  size_t minPts = 20;
  size_t min_pts_per_cluster = 10;
  size_t max_pts_per_cluster = source->size();
  for (size_t i = 0; i < source->points.size(); i++) {
    if (types[i] == PROCESSED) {
      continue;
    }
    size_t nn_size = kdtree_->radiusSearch(i, eps, nn_indices, nn_distances);
    if (nn_size < minPts) {
      is_noise[i] = true;
      continue;
    }
    std::vector<int> seed_queue;
    seed_queue.push_back(i);
    types[i] = PROCESSED;
    for (size_t j = 0; j < nn_size; j++) {
      if (nn_indices[j] != int(i)) {
        seed_queue.push_back(nn_indices[j]);
        types[nn_indices[j]] = PROCESSING;
      }
    }  // for every point near the chosen core point.
    size_t sq_idx = 1;
    while (sq_idx < seed_queue.size()) {
      int cloud_index = seed_queue[sq_idx];
      if (is_noise[cloud_index] || types[cloud_index] == PROCESSED) {
        // seed_queue.push_back(cloud_index);
        types[cloud_index] = PROCESSED;
        sq_idx++;
        continue;  // no need to check neighbors.
      }
      nn_size =
          kdtree_->radiusSearch(cloud_index, eps, nn_indices, nn_distances);
      if (nn_size >= minPts) {
        for (size_t j = 0; j < nn_size; j++) {
          if (types[nn_indices[j]] == UN_PROCESSED) {
            seed_queue.push_back(nn_indices[j]);
            types[nn_indices[j]] = PROCESSING;
          }
        }
      }
      types[cloud_index] = PROCESSED;
      sq_idx++;
    }
    if (seed_queue.size() >= min_pts_per_cluster &&
        seed_queue.size() <= max_pts_per_cluster) {
      for (size_t j = 0; j < seed_queue.size(); ++j) {
        result->emplace_back(source->points[seed_queue[j]]);
      }
      result->clear();
    }
  }
}

void Operator::region_growing_cluster(CloudPtr source, CloudPtr &result) {
  inno_log_info("Region Growing Cluster Input size : %ld", source->size());
  result->clear();
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  normal_estimator_.setInputCloud(source);
  normal_estimator_.compute(*normals);

  pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
  int npoint = source->size();

  reg.setMinClusterSize(0.1 * npoint);
  reg.setMaxClusterSize(npoint);
  reg.setSearchMethod(tree_);
  reg.setNumberOfNeighbours(50);
  reg.setInputCloud(source);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(smoothness_ / 180.0 * M_PI);
  reg.setCurvatureThreshold(curvature_);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);
  inno_log_info("Region_growing_cluster size %ld", clusters.size());

  // find the most voluminous cluster
  int index = 0;
  size_t max_element = 0;
  for (size_t i = 0; i < clusters.size(); ++i) {
    if (clusters[i].indices.size() > max_element) {
      index = i;
      max_element = clusters[i].indices.size();
    }
  }
  for (size_t j = 0; j < max_element; ++j) {
    result->emplace_back(source->points[clusters[index].indices[j]]);
  }
}

void Operator::voteNormal(CloudPtr source, CloudPtr &result, float &angle) {
  result->clear();
  kdtree_->setInputCloud(source);
  std::vector<Ballot> ballots{};
  for (auto &point : source->points) {
    std::vector<int> index(2);
    std::vector<float> distance(2);
    kdtree_->nearestKSearch(point, 2, index, distance);
    auto dp = source->points[index[1]];
    Ballot np;
    np.x = point.x - dp.x;  // 0
    np.y = point.y - dp.y;  // 0
    np.z = point.z - dp.z;  // 1
    float norme = sqrt(np.x * np.x + np.y * np.y + np.z * np.z);
    if (norme > 0 && norme < 0.1) {
      np.x = np.x / norme;
      np.y = np.y / norme;
      np.z = np.z / norme;
      float angle = acosf(np.z);
      if (angle > M_PI / 2) {
        angle = M_PI - angle;
        np.x *= -1;
        np.y *= -1;
        np.z *= -1;
      }
      np.angle = angle;
      ballots.emplace_back(np);
    }
  }

  std::sort(
      ballots.begin(), ballots.end(),
      [&](const Ballot &a, const Ballot &b) { return (a.angle < b.angle); });
  size_t id_ballot = ballots.size() / 3;
  float standard_max = 1.2 * ballots[id_ballot].angle;
  float standard_min = 0.8 * ballots[id_ballot].angle;
  inno_log_info("Reference vote angle %f", ballots[id_ballot].angle);
  Ballot sommus{};
  int account = 0;
  for (auto &item : ballots) {
    if (item.angle < standard_max && item.angle > standard_min) {
      sommus.x += item.x;
      sommus.y += item.y;
      sommus.z += item.z;
      sommus.angle += item.angle;
      ++account;
    }
  }

  sommus.x /= account;
  sommus.y /= account;
  sommus.z /= account;
  sommus.angle /= account;
  angle = sommus.angle;
  inno_log_info("Average Normal angle %f", sommus.angle);

  std::vector<float> rotate(9, 0);
  std::vector<float> trans(3, 0);
  std::vector<float> nor = {sommus.x, sommus.y, sommus.z, 0};
  std::vector<float> init = {0.0, 0.0, 1.0, 0.0};
  trans_plane_plane_3d(nor, init, rotate, trans);
  float z_min = std::numeric_limits<float>::max();
  float z_max = -std::numeric_limits<float>::max();

  for (auto &point : source->points) {
    auto plat_point = point;
    plat_point.x = point.x * rotate[0] + point.y * rotate[1] +
                   point.z * rotate[2] + trans[0];
    plat_point.y = point.x * rotate[3] + point.y * rotate[4] +
                   point.z * rotate[5] + trans[1];
    plat_point.z = point.x * rotate[6] + point.y * rotate[7] +
                   point.z * rotate[8] + trans[2];
    z_min = std::min(z_min, plat_point.z);
    z_max = std::max(z_max, plat_point.z);
    result->emplace_back(plat_point);
  }

  float z_step = 0.01;
  int z_len = std::ceil((z_max - z_min) / z_step);
  std::vector<int> z_distrubution(z_len, 0);

  for (auto &point : result->points) {
    int ncell = std::round((point.z - z_min) / z_step);
    ++z_distrubution[ncell];
  }

  auto iter = std::max_element(z_distrubution.begin(), z_distrubution.end());
  float plane_z = z_min + (iter - z_distrubution.begin()) * z_step;
  for (auto &point : result->points) {
    point.z = plane_z;
  }
}

void Operator::voteDistance(CloudPtr reference, CloudPtr target,
                            CloudPtr &result) {
  result->clear();
  float precision = 0.005;
  float range = 1;
  int step = range / precision;
  std::vector<int> vote_map(step, 0);

  for (auto point : target->points) {
    float tempd = a_ * point.x + b_ * point.y + c_ * point.z;
    if (tempd > -d_) {
      int index = (tempd + d_) / precision;
      ++vote_map[index];
    }
  }
  auto iter = std::max_element(vote_map.begin(), vote_map.end());
  inno_log_info("[Vote Distance] Origin d_ = %f ", d_);
  float final_d = d_ - (iter - vote_map.begin()) * precision;
  inno_log_info("[Vote Distance] Max_elemet dm = %f ", final_d);
  float distance = d_ - final_d;
  inno_log_info("[Vote Distance] Deplacement distance = %f", distance);
  d_ = final_d;
  pcl::PointXYZ deplacement;
  deplacement.x = distance * a_;
  deplacement.y = distance * b_;
  deplacement.z = distance * c_;
  inno_log_info("[Vote Distance] find deplacement (%f, %f, %f)", deplacement.x,
                deplacement.y, deplacement.z);
  for (auto point : reference->points) {
    pcl::PointXYZI tempp;
    tempp.x = point.x + deplacement.x;
    tempp.y = point.y + deplacement.y;
    tempp.z = point.z + deplacement.z;
    tempp.intensity = 254;
    result->emplace_back(tempp);
  }
}

void Operator::fitPlane(CloudPtr source, CloudPtr &result, float &a, float &b,
                        float &c, float &d) {
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  CloudPtr plane_cloud = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::SACSegmentation<pcl::PointXYZI> seg;

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(40000);
  seg.setDistanceThreshold(0.008);
  seg.setInputCloud(source);
  seg.segment(*inliers_plane, *coefficients_plane);

  a = coefficients_plane->values[0];
  b = coefficients_plane->values[1];
  c = coefficients_plane->values[2];
  d = coefficients_plane->values[3];
  inno_log_info("Plane coefficient (a,b,c,d) = (%f, %f, %f, %f)", a, b, c, d);
  pcl::copyPointCloud(*source, *inliers_plane, *result);
}

void Operator::fitCircle(CloudPtr source, float &center_x, float &center_y,
                         float &center_z, float &radius) {
  cv::Point2f center;
  center_z = 0;
  std::vector<cv::Point2f> input;
  for (auto &point : source->points) {
    cv::Point2f temp(point.x, point.y);
    input.emplace_back(temp);
    center_z += point.z;
  }
  cv::minEnclosingCircle(input, center, radius);
  center_x = center.x;
  center_y = center.y;
  center_z = center_z / source->size();
  cv::Mat paint = cv::Mat(500, 500, CV_8UC3, cv::Scalar::all(0));
  cv::circle(paint, cv::Point2f(250, 250), radius * 500,
             cv::Scalar(0, 0, 255, 255));
  for (auto &point : source->points) {
    cv::circle(paint,
               cv::Point2f(500 * (point.x - center.x) + 250,
                           500 * (point.y - center.y) + 250),
               0.1, cv::Scalar(0, 255, 0, 255));
  }
  cv::imwrite(debug_folder_ + frame_name_ + "_circle.png", paint);
}

void Operator::findPlate(CloudPtr source, CloudPtr &result,
                         const pcl::Normal &reference) {
  result->clear();
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  normal_estimator_.setInputCloud(source);
  normal_estimator_.compute(*normals);

  for (size_t i = 0; i < normals->size(); ++i) {
    float pscalar = sqrt(abs(normals->points[i].normal_x * reference.normal_x +
                             normals->points[i].normal_y * reference.normal_y +
                             normals->points[i].normal_z * reference.normal_z));
    if (pscalar > 0.92) {
      result->emplace_back(source->points[i]);
    }
  }
}

Eigen::Vector3f Operator::find_feature_point(CloudPtr source) {
  CloudPtr projection_plane = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);

  Eigen::Matrix3f pure_rotation = transforme_normal_normal_3d(
      Eigen::Vector3f(a_, b_, c_), Eigen::Vector3f(0, 0, 1));
  Eigen::Matrix3f inverse_rotation = pure_rotation.inverse();

  for (auto point : source->points) {
    Eigen::Vector3f curr =
        pure_rotation * Eigen::Vector3f{point.x, point.y, point.z};
    pcl::PointXYZI tempp(point.intensity);
    tempp.x = curr.x();
    tempp.y = curr.y();
    tempp.z = curr.z();
    tempp.intensity = point.intensity;
    projection_plane->emplace_back(tempp);
  }
  float center_x, center_y, center_z, radius;
  fitCircle(projection_plane, center_x, center_y, center_z, radius);
  save_pcd(debug_folder_ + frame_name_ + "_p.pcd", projection_plane);
  Eigen::Vector3f selected_point =
      inverse_rotation * Eigen::Vector3f{center_x, center_y, center_z};
  return selected_point;
}

void Operator::mergePointCloud(CloudPtr source, CloudPtr &result) {
  for (auto point : source->points) {
    result->emplace_back(point);
  }
}

void Operator::nearMarkable(CloudPtr source, CloudPtr &result) {
  result->clear();
  kdtree_->setInputCloud(source);
  for (size_t i = 0; i < source->size(); ++i) {
    if (source->points[i].intensity != 254) {
      std::vector<int> indexes;
      std::vector<float> distances;
      kdtree_->radiusSearch(i, 0.2, indexes, distances);
      if (indexes.size() > 10) {
        for (auto id : indexes) {
          if (source->points[id].intensity == 254) {
            result->emplace_back(source->points[i]);
            break;
          }
        }
      }
    }
  }
}

int Operator::compute(Eigen::Vector3f &feature_point, CloudPtr &output) {
  CloudPtr around_cloud = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  CloudPtr attension_cloud = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  CloudPtr markable_cloud = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  CloudPtr cluster_cloud = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  CloudPtr attension_plane = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  CloudPtr around_plane = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  CloudPtr cleared_around_plane = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
  CloudPtr final_plane = CloudPtr(new pcl::PointCloud<pcl::PointXYZI>);

  if (input_cloud_->size()) {
    findAround(input_cloud_, around_cloud, attension_cloud, 254);
    save_pcd(debug_folder_ + frame_name_ + "_1.pcd", around_cloud);
    save_pcd(debug_folder_ + frame_name_ + "_2.pcd", attension_cloud);
  } else {
    inno_log_warning("Input PointCLoud is NULL");
    return CE_INPUT_NULL;
  }

  if (lidar_model_ == JAGUAR) {
    if (attension_cloud->size()) {
      guide_filter(attension_cloud, markable_cloud);
      save_pcd(debug_folder_ + frame_name_ + "_3.pcd", markable_cloud);
      region_growing_cluster(markable_cloud, cluster_cloud);
      save_pcd(debug_folder_ + frame_name_ + "_4.pcd", cluster_cloud);
    } else {
      return CE_ATTENSION_NULL;
    }
    if (cluster_cloud->size()) {
      fitPlane(cluster_cloud, attension_plane, a_, b_, c_, d_);
      save_pcd(debug_folder_ + frame_name_ + "_5.pcd", attension_plane);
      findPlate(around_cloud, around_plane, pcl::Normal(a_, b_, c_));
      mergePointCloud(cluster_cloud, around_plane);
      nearMarkable(around_plane, cleared_around_plane);
      voteDistance(attension_plane, cleared_around_plane, final_plane);
    } else {
      return CE_CLUSTER_FAIL;
    }

    if (final_plane->size()) {
      feature_point = find_feature_point(final_plane);
      output = final_plane;
      return 0;
    }
  } else if (lidar_model_ = FALCON) {
    if (attension_cloud->size()) {
      guide_filter(attension_cloud, markable_cloud);
      save_pcd(debug_folder_ + frame_name_ + "_3.pcd", markable_cloud);
      outline_remove(markable_cloud, cluster_cloud);
      save_pcd(debug_folder_ + frame_name_ + "_4.pcd", cluster_cloud);
    } else {
      return CE_ATTENSION_NULL;
    }
    if (cluster_cloud->size()) {
      fitPlane(cluster_cloud, attension_plane, a_, b_, c_, d_);
      save_pcd(debug_folder_ + frame_name_ + "_5.pcd", attension_plane);
    } else {
      return CE_CLUSTER_FAIL;
    }

    if (attension_plane->size()) {
      feature_point = find_feature_point(attension_plane);
      output = attension_plane;
      return 0;
    }
  }
  return CE_NO_RESULT;
}
}  // namespace Calibration
