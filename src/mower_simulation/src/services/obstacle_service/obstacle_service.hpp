/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef OBSTACLE_SERVICE_HPP
#define OBSTACLE_SERVICE_HPP

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <ObstacleServiceBase.hpp>

// Simulated obstacle sensor (e.g. a bumper). The simulator has no collision physics, so it reports
// whatever contact point (x, y in base_link, metres) is published on ~report_contact.
class ObstacleService : public ObstacleServiceBase {
 public:
  ObstacleService(uint16_t service_id, ros::NodeHandle& private_nh);

 private:
  void OnReportContact(const geometry_msgs::Point::ConstPtr& contact);

  ros::Subscriber contact_sub_;
};

#endif  // OBSTACLE_SERVICE_HPP
