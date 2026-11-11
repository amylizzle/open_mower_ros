/*
 * Copyright (C) 2026 The OpenMower Contributors
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "obstacle_service.hpp"

ObstacleService::ObstacleService(uint16_t service_id, ros::NodeHandle& private_nh) : ObstacleServiceBase(service_id) {
  contact_sub_ = private_nh.subscribe("report_contact", 10, &ObstacleService::OnReportContact, this);
}

void ObstacleService::OnReportContact(const geometry_msgs::Point::ConstPtr& contact) {
  const float point[2] = {static_cast<float>(contact->x), static_cast<float>(contact->y)};
  StartTransaction();
  SendObstacleContact(point, 2);
  CommitTransaction();
}
