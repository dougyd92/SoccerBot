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

#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <rclcpp/rclcpp.hpp>
# include <ament_index_cpp/get_package_share_directory.hpp>

# include <rcl_interfaces/msg/parameter_event.hpp>
# include <std_srvs/srv/empty.hpp>
# include <soccer_sim/srv/spawn.hpp>
# include <soccer_sim/srv/kill.hpp>
# include <map>

# include "turtle.h"
#endif

namespace soccer_sim
{

class SimulationFrame : public QFrame
{
  Q_OBJECT
public:
  SimulationFrame(rclcpp::Node::SharedPtr& node_handle, QWidget* parent = 0, Qt::WindowFlags f = Qt::WindowFlags());
  ~SimulationFrame();

  std::string spawnAgent(const std::string& name, float x, float y, float angle);
  std::string spawnAgent(const std::string& name, float x, float y, float angle, size_t index);

protected:
  void paintEvent(QPaintEvent* event);

private slots:
  void onUpdate();

private:
  void updateTurtles();
  bool hasTurtle(const std::string& name);

  bool resetCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);
  bool spawnCallback(const soccer_sim::srv::Spawn::Request::SharedPtr, soccer_sim::srv::Spawn::Response::SharedPtr);
  bool killCallback(const soccer_sim::srv::Kill::Request::SharedPtr, soccer_sim::srv::Kill::Response::SharedPtr);

  void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr);

  rclcpp::Node::SharedPtr nh_;

  QTimer* update_timer_;

  uint64_t frame_count_;

  rclcpp::Time last_turtle_update_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::Service<soccer_sim::srv::Spawn>::SharedPtr spawn_srv_;
  rclcpp::Service<soccer_sim::srv::Kill>::SharedPtr kill_srv_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  typedef std::map<std::string, AgentPtr> M_Agent;
  M_Agent turtles_;
  uint32_t id_counter_;

  QVector<QImage> turtle_images_;

  float meter_;
  float width_in_meters_;
  float height_in_meters_;
};

}
