#include "ObstacleServiceInterface.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

namespace {
constexpr double kCell = 0.05;          // [m] storage grid, the local costmap resolution
constexpr double kHeight = 0.1;         // [m] z of firmware obstacles, inside the obstacle layer's height window
constexpr double kPublishPeriod = 0.5;  // [s]
}  // namespace

ObstacleServiceInterface::ObstacleServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx,
                                                   ros::NodeHandle& nh, const ros::NodeHandle& param_nh)
    : ObstacleServiceInterfaceBase(service_id, ctx),
      radius_(param_nh.param("services/obstacle/radius", 0.2)),
      frame_(param_nh.param<std::string>("services/obstacle/frame", "map")) {
  RebuildCloud();
  points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/obstacles/points", 1);
  new_sub_ = nh.subscribe("/obstacles/new", 10, &ObstacleServiceInterface::OnNewObstacle, this);
  clear_sub_ = nh.subscribe("/obstacles/clear", 1, &ObstacleServiceInterface::OnClear, this);
  publish_timer_ = nh.createTimer(ros::Duration(kPublishPeriod), [this](const ros::TimerEvent&) { Publish(); });
}

void ObstacleServiceInterface::OnObstacleContactChanged(const float* new_value, uint32_t length) {
  if (length != 2) return;
  const double contact_x = new_value[0];
  const double contact_y = new_value[1];

  // Put the disk's centre one radius beyond the contact point, away from the robot, so the disk
  // starts where the robot touched it and does not cover the robot itself.
  double dir_x = 1.0, dir_y = 0.0;
  const double distance = std::hypot(contact_x, contact_y);
  if (distance > 1e-3) {
    dir_x = contact_x / distance;
    dir_y = contact_y / distance;
  }
  const double centre_x = contact_x + dir_x * radius_;
  const double centre_y = contact_y + dir_y * radius_;

  std::vector<tf2::Vector3> points;
  const int steps = static_cast<int>(radius_ / kCell);
  for (int i = -steps; i <= steps; ++i) {
    for (int j = -steps; j <= steps; ++j) {
      if (std::hypot(i * kCell, j * kCell) <= radius_) {
        points.emplace_back(centre_x + i * kCell, centre_y + j * kCell, kHeight);
      }
    }
  }
  ROS_INFO_STREAM("Obstacle contact at (" << contact_x << ", " << contact_y << ") in base_link, marking a " << radius_
                                          << " m obstacle");
  // The contact has just happened, so use the latest known robot pose (ros::Time(0)) rather than
  // waiting for one stamped after it.
  AddObstacle("base_link", ros::Time(0), points);
}

void ObstacleServiceInterface::OnNewObstacle(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
  std::vector<tf2::Vector3> points;
  sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x"), y(*cloud, "y"), z(*cloud, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) points.emplace_back(*x, *y, *z);
  }
  AddObstacle(cloud->header.frame_id, cloud->header.stamp, points);
}

void ObstacleServiceInterface::OnClear(const std_msgs::Empty::ConstPtr&) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cells_.clear();
    RebuildCloud();
  }
  ROS_INFO("Obstacles: forgot all obstacles");
}

void ObstacleServiceInterface::AddObstacle(const std::string& frame, const ros::Time& stamp,
                                           const std::vector<tf2::Vector3>& points) {
  if (points.empty()) return;

  tf2::Transform to_frame;
  try {
    tf2::fromMsg(tf_buffer_.lookupTransform(frame_, frame, stamp, ros::Duration(0.2)).transform, to_frame);
  } catch (const tf2::TransformException& e) {
    ROS_WARN_STREAM("Obstacles: dropping obstacle in frame '" << frame << "': " << e.what());
    return;
  }

  tf2::Vector3 centre(0, 0, 0);
  size_t total;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& point : points) {
      const tf2::Vector3 p = to_frame * point;
      centre += p;
      cells_.insert({static_cast<int32_t>(std::lround(p.x() / kCell)), static_cast<int32_t>(std::lround(p.y() / kCell)),
                     static_cast<int32_t>(std::lround(p.z() / kCell))});
    }
    RebuildCloud();
    total = cells_.size();
  }
  centre /= static_cast<double>(points.size());
  ROS_INFO("Obstacles: new obstacle at (%.2f, %.2f) in '%s', %zu obstacle points in total", centre.x(), centre.y(),
           frame_.c_str(), total);

  // Don't wait for the next periodic publish: after a bump, every fraction of a second the robot
  // keeps driving counts.
  Publish();
}

void ObstacleServiceInterface::RebuildCloud() {
  cloud_.header.frame_id = frame_;
  sensor_msgs::PointCloud2Modifier modifier(cloud_);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(cells_.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud_, "x"), y(cloud_, "y"), z(cloud_, "z");
  for (const auto& cell : cells_) {
    *x = static_cast<float>(cell[0] * kCell);
    *y = static_cast<float>(cell[1] * kCell);
    *z = static_cast<float>(cell[2] * kCell);
    ++x, ++y, ++z;
  }
}

void ObstacleServiceInterface::Publish() {
  std::lock_guard<std::mutex> lock(mutex_);
  cloud_.header.stamp = ros::Time::now();
  points_pub_.publish(cloud_);
}
