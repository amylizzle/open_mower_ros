#ifndef OBSTACLESERVICEINTERFACE_H
#define OBSTACLESERVICEINTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ObstacleServiceInterfaceBase.hpp>
#include <array>
#include <mutex>
#include <set>
#include <string>
#include <vector>

/**
 * Keeps every known obstacle and publishes them all for the costmap on /obstacles/points.
 *
 * Obstacles come from two places:
 * - The firmware's ObstacleService: a point in base_link where the robot touched something, e.g.
 *   a bumper. The firmware only says where; how big to assume the obstacle is, is configured here
 *   (services/obstacle/radius, default 0.2 m). It becomes a filled disk with its near edge at the
 *   contact point.
 * - Anything else (a camera, the simulator's sim_obstacles.py): one PointCloud2 per new obstacle on
 *   /obstacles/new, in any frame tf can transform to services/obstacle/frame (default map) at the
 *   message's timestamp.
 *
 * Each obstacle is stored in that frame as it was when reported, so it stays where it was found
 * however the robot moves afterwards. Points are kept on the costmap's grid, so reporting the same
 * spot again adds nothing. All of them are published whenever an obstacle is added, and every
 * 0.5 s. Others may publish on /obstacles/points directly too.
 *
 * Publish an empty message on /obstacles/clear to forget all obstacles. Cells the costmap has
 * already marked stay marked until /move_base_flex/clear_costmaps is called.
 */
class ObstacleServiceInterface : public ObstacleServiceInterfaceBase {
 public:
  ObstacleServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, ros::NodeHandle& nh,
                           const ros::NodeHandle& param_nh);

 protected:
  void OnObstacleContactChanged(const float* new_value, uint32_t length) override;

 private:
  using Cell = std::array<int32_t, 3>;  // a point on the storage grid

  void OnNewObstacle(const sensor_msgs::PointCloud2::ConstPtr& cloud);
  void OnClear(const std_msgs::Empty::ConstPtr&);
  // Stores `points` (given in `frame` at `stamp`) in frame_, then publishes all obstacles.
  void AddObstacle(const std::string& frame, const ros::Time& stamp, const std::vector<tf2::Vector3>& points);
  void RebuildCloud();  // call with mutex_ held
  void Publish();

  const double radius_;      // [m] assumed size of an obstacle the firmware touched
  const std::string frame_;  // frame obstacles are stored and published in

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  ros::Publisher points_pub_;
  ros::Subscriber new_sub_;
  ros::Subscriber clear_sub_;
  ros::Timer publish_timer_;

  std::mutex mutex_;
  std::set<Cell> cells_;
  sensor_msgs::PointCloud2 cloud_;  // cells_ as a message, rebuilt only when they change
};

#endif  // OBSTACLESERVICEINTERFACE_H
