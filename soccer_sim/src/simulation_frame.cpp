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

#include "soccer_sim/simulation_frame.h"
#include "soccer_sim/constants.h"

#include <QPointF>

#include <cstdlib>
#include <ctime>

#define DEFAULT_BG_R 0x07
#define DEFAULT_BG_G 0x8c
#define DEFAULT_BG_B 0x2d

namespace soccer_sim
{

/*
  Node for the simulation environment.
  This is essentially turtlesim_node, albeit heavily modified.
  Contains all the agents, which each have a /pose and /cmd_vel topic
  Updates the positions of each agent at each time step and renders them.
*/
SimulationFrame::SimulationFrame(rclcpp::Node::SharedPtr& node_handle, QWidget* parent, Qt::WindowFlags f)
: QFrame(parent, f)
, frame_count_(0)
, id_counter_(0)
{
  setFixedSize(68*METERS_TO_PX, 105*METERS_TO_PX);
  setWindowTitle("Soccer Bots");

  srand(time(NULL));

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  nh_ = node_handle;
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = 0;
  range.step = 1;
  range.to_value = 255;
  rcl_interfaces::msg::ParameterDescriptor background_r_descriptor;
  background_r_descriptor.description = "Red channel of the background color";
  background_r_descriptor.integer_range.push_back(range);
  rcl_interfaces::msg::ParameterDescriptor background_g_descriptor;
  background_g_descriptor.description = "Green channel of the background color";
  background_g_descriptor.integer_range.push_back(range);
  rcl_interfaces::msg::ParameterDescriptor background_b_descriptor;
  background_b_descriptor.description = "Blue channel of the background color";
  background_b_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("background_r", rclcpp::ParameterValue(DEFAULT_BG_R), background_r_descriptor);
  nh_->declare_parameter("background_g", rclcpp::ParameterValue(DEFAULT_BG_G), background_g_descriptor);
  nh_->declare_parameter("background_b", rclcpp::ParameterValue(DEFAULT_BG_B), background_b_descriptor);

  QString images_path = (ament_index_cpp::get_package_share_directory("soccer_sim") + "/images/").c_str();
  QVector<QString> team_a_image_names;
  QVector<QString> team_b_image_names;
  QImage img;
  team_a_image_names.append("jersey_A.png");
  team_b_image_names.append("jersey_B.png");
  for (int i = 0; i < team_a_image_names.size(); ++i)
  {
    img.load(images_path + team_a_image_names[i]);
    team_a_images_.append(img);
  }
  for (int i = 0; i < team_b_image_names.size(); ++i)
  {
    img.load(images_path + team_b_image_names[i]);
    team_b_images_.append(img);
  }
  img.load(images_path + "ball.png");
  ball_image_ = img;

  reset_srv_ = nh_->create_service<std_srvs::srv::Empty>("reset", std::bind(&SimulationFrame::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
  spawn_srv_ = nh_->create_service<soccer_sim::srv::Spawn>("spawn", std::bind(&SimulationFrame::spawnCallback, this, std::placeholders::_1, std::placeholders::_2));
  kill_srv_ = nh_->create_service<soccer_sim::srv::Kill>("kill", std::bind(&SimulationFrame::killCallback, this, std::placeholders::_1, std::placeholders::_2));

  rclcpp::QoS qos(rclcpp::KeepLast(100), rmw_qos_profile_sensor_data);
  parameter_event_sub_ = nh_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", qos, std::bind(&SimulationFrame::parameterEventCallback, this, std::placeholders::_1));

  RCLCPP_INFO(nh_->get_logger(), "Starting soccer_sim with node name %s", nh_->get_fully_qualified_name());

  width_in_meters_ = (width() - 1) / METERS_TO_PX;
  height_in_meters_ = (height() - 1) / METERS_TO_PX;
}

SimulationFrame::~SimulationFrame()
{
  delete update_timer_;
}

bool SimulationFrame::spawnCallback(const soccer_sim::srv::Spawn::Request::SharedPtr req, soccer_sim::srv::Spawn::Response::SharedPtr res)
{
  int agent_type;
  if (req->agent_type ==  "TEAM_A") 
    agent_type = AgentType::TEAM_A;
  else if (req->agent_type ==  "TEAM_B") 
    agent_type = AgentType::TEAM_B;
  else if (req->agent_type ==  "BALL") 
    agent_type = AgentType::BALL;
  else {
    RCLCPP_ERROR(nh_->get_logger(), "Invalid agent_type specified: %s", req->agent_type.c_str());
    return false;
  }

  std::string name = spawnAgent(req->name, req->x, req->y, agent_type);
  
  if (name.empty())
  {
    RCLCPP_ERROR(nh_->get_logger(), "An agent named [%s] already exists", req->name.c_str());
    return false;
  }

  res->name = name;

  return true;
}

bool SimulationFrame::killCallback(const soccer_sim::srv::Kill::Request::SharedPtr req, soccer_sim::srv::Kill::Response::SharedPtr)
{
  M_Agent::iterator it = agents_.find(req->name);
  if (it == agents_.end())
  {
    RCLCPP_ERROR(nh_->get_logger(), "Tried to kill agent [%s], which does not exist", req->name.c_str());
    return false;
  }

  agents_.erase(it);
  update();

  return true;
}

void SimulationFrame::parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event)
{
  // only consider events from this node
  if (event->node == nh_->get_fully_qualified_name())
  {
    // since parameter events for this event aren't expected frequently just always call update()
    update();
  }
}

bool SimulationFrame::hasAgent(const std::string& name)
{
  return agents_.find(name) != agents_.end();
}

std::string SimulationFrame::spawnAgent(const std::string& name, float x, float y, int agent_type)
{
  std::string real_name = name;
  if (real_name.empty())
  {
    do
    {
      std::stringstream ss;
      ss << "agent" << ++id_counter_;
      real_name = ss.str();
    } while (hasAgent(real_name));
  }
  else
  {
    if (hasAgent(real_name))
    {
      return "";
    }
  }

  QImage image;
  switch(agent_type) {
    case AgentType::TEAM_A:
      image = team_a_images_[0];
      break;
    case AgentType::TEAM_B:
      image = team_b_images_[0];
      break;
    case AgentType::BALL:
      image = ball_image_;
      break;
  }

  AgentPtr t = std::make_shared<Agent>(nh_, real_name, image, QPointF(x, height_in_meters_ - y));
  agents_[real_name] = t;
  update();

  RCLCPP_INFO(nh_->get_logger(), "Spawning agent [%s] at x=[%f], y=[%f]", real_name.c_str(), x, y);

  return real_name;
}

void SimulationFrame::onUpdate()
{
  if (!rclcpp::ok())
  {
    close();
    return;
  }

  rclcpp::spin_some(nh_);

  updateAgents();
}

void SimulationFrame::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;
  nh_->get_parameter("background_r", r);
  nh_->get_parameter("background_g", g);
  nh_->get_parameter("background_b", b);
  QRgb background_color = qRgb(r, g, b);
  painter.fillRect(0, 0, width(), height(), background_color);

  M_Agent::iterator it = agents_.begin();
  M_Agent::iterator end = agents_.end();
  for (; it != end; ++it)
  {
    it->second->paint(painter);
  }
}

void SimulationFrame::updateAgents()
{
  if (last_agent_update_.nanoseconds() == 0)
  {
    last_agent_update_ = nh_->now();
    return;
  }

  bool modified = false;
  M_Agent::iterator it = agents_.begin();
  M_Agent::iterator end = agents_.end();
  for (; it != end; ++it)
  {
    modified |= it->second->update(0.001 * update_timer_->interval(), width_in_meters_, height_in_meters_);
  }
  if (modified)
  {
    update();
  }

  ++frame_count_;
}

bool SimulationFrame::resetCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "Resetting soccer_sim.");
  agents_.clear();
  id_counter_ = 0;
  update();
  return true;
}

}
