/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "soccer_sim/agent.h"
#include "soccer_sim/constants.h"

#include <math.h>

#include <QRgb>

namespace soccer_sim
{

static double normalizeAngle(double angle)
{
  return angle - (TWO_PI * std::floor((angle + PI) / (TWO_PI)));
}

Agent::Agent(rclcpp::Node::SharedPtr& nh, const std::string& real_name, const QImage& agent_image, const QPointF& pos)
: nh_(nh)
, agent_name(real_name)
, agent_image_(agent_image)
, pos_(pos)
, orient_(0.0)
, lin_vel_x_(0.0)
, lin_vel_y_(0.0)
, ang_vel_(0.0)
{
  rclcpp::QoS qos(rclcpp::KeepLast(7));
  velocity_sub_ = nh_->create_subscription<geometry_msgs::msg::Twist>(real_name + "/cmd_vel", qos, std::bind(&Agent::velocityCallback, this, std::placeholders::_1));
  pose_pub_ = nh_->create_publisher<soccer_sim::msg::Pose>(real_name + "/pose", qos);
  teleport_absolute_srv_ = nh_->create_service<soccer_sim::srv::TeleportAbsolute>(real_name + "/teleport_absolute", std::bind(&Agent::teleportAbsoluteCallback, this, std::placeholders::_1, std::placeholders::_2));

  last_command_time_ = nh_->now();
}


void Agent::velocityCallback(const geometry_msgs::msg::Twist::ConstSharedPtr vel)
{
  last_command_time_ = nh_->now();
  lin_vel_x_ = vel->linear.x;
  lin_vel_y_ = vel->linear.y;
  ang_vel_ = vel->angular.z;
}

bool Agent::teleportAbsoluteCallback(const soccer_sim::srv::TeleportAbsolute::Request::SharedPtr req, soccer_sim::srv::TeleportAbsolute::Response::SharedPtr)
{
  teleport_requests_.push_back(TeleportRequest(req->x, req->y, req->theta));
  return true;
}

bool Agent::update(double dt, qreal canvas_width, qreal canvas_height)
{
  bool modified = false;

  // first process any teleportation requests, in order
  V_TeleportRequest::iterator it = teleport_requests_.begin();
  V_TeleportRequest::iterator end = teleport_requests_.end();
  for (; it != end; ++it)
  {
    const TeleportRequest& req = *it;

    pos_.setX(req.pos.x());
    pos_.setY(std::max(0.0, static_cast<double>(canvas_height - req.pos.y())));
    orient_ = req.theta;

    modified = true;
  }

  teleport_requests_.clear();

  if (nh_->now() - last_command_time_ > rclcpp::Duration(1.0, 0))
  {
    lin_vel_x_ = 0.0;
    lin_vel_y_ = 0.0;
    ang_vel_ = 0.0;
  }

  QPointF old_pos = pos_;

  orient_ = orient_ + ang_vel_ * dt;
  // Keep orient_ between -pi and +pi
  orient_ = normalizeAngle(orient_);
  pos_.rx() += std::cos(orient_) * lin_vel_x_ * dt
             - std::sin(orient_) * lin_vel_y_ * dt;
  pos_.ry() -= std::cos(orient_) * lin_vel_y_ * dt
             + std::sin(orient_) * lin_vel_x_ * dt;

  // Clamp to screen size
  if (pos_.x() < 0 || pos_.x() > canvas_width ||
      pos_.y() < 0 || pos_.y() > canvas_height)
  {
    RCLCPP_WARN(nh_->get_logger(), "Oh no! I hit the wall! (Clamping from [x=%f, y=%f])", pos_.x(), pos_.y());
  }

  pos_.setX(std::min(std::max(static_cast<double>(pos_.x()), 0.0), static_cast<double>(canvas_width)));
  pos_.setY(std::min(std::max(static_cast<double>(pos_.y()), 0.0), static_cast<double>(canvas_height)));

  // Publish pose of the agent
  auto p = std::make_unique<soccer_sim::msg::Pose>();
  p->x = pos_.x();
  p->y = canvas_height - pos_.y();
  p->name = agent_name;
  p->linear_velocity = std::sqrt(lin_vel_x_ * lin_vel_x_ + lin_vel_y_ * lin_vel_y_);
  p->angular_velocity = ang_vel_;
  pose_pub_->publish(std::move(p));

  RCLCPP_DEBUG(nh_->get_logger(), "[%s]: pos_x: %f pos_y: %f theta: %f", nh_->get_namespace(), pos_.x(), pos_.y(), orient_);

  if (pos_ != old_pos)
  {
    modified = true;
  }

  return modified;
}

void Agent::paint(QPainter& painter)
{
  QPointF p = pos_ * METERS_TO_PX;
  p.rx() -= 0.5 * agent_image_.width();
  p.ry() -= 0.5 * agent_image_.height();
  painter.drawImage(p, agent_image_);
}

}
