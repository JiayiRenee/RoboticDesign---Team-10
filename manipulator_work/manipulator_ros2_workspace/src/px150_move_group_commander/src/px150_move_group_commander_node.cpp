// Copyright 2022 Trossen Robotics
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "interbotix_moveit_interface/moveit_interface_obj.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <memory>
#include "shape_msgs/msg/solid_primitive.hpp"

#include <memory>


#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/trigger.hpp"
using std::placeholders::_1;






int main(int argc, char ** argv)
{
  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0;
  box_pose.position.y = 0;
  box_pose.position.z = -0.320/2;
  box_pose.orientation.x = 0;
  box_pose.orientation.y = 0;
  box_pose.orientation.z = 0;
  box_pose.orientation.w = 1;
 
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("px150_move_group_commander_node", node_options);
  auto moveit_interface = std::make_shared<interbotix::InterbotixMoveItInterface>(node);
  moveit_interface->moveit_add_collision_box(box_pose,0.425,0.488,0.320);
  moveit_interface->moveit_plan_ee_position(0.3,0.05,-0.128);
  moveit_interface->moveit_execute_plan();


  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);

  


  executor->spin();

  



  rclcpp::shutdown();
  return 0;
}