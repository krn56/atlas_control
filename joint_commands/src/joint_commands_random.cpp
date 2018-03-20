/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>

ros::Publisher pub_joint_commands_;
osrf_msgs::JointCommands jointcommands;
static float x1 = 0, x2 = 0;
double stand_prep_msg[28] = {2.438504816382192e-05, 0.0015186156379058957, 9.983908967114985e-06, -0.0010675729718059301,
 -0.0003740221436601132, 0.1, -0.2333149015903473, 0.5181407332420349, -0.27610817551612854,
 -0.062101610004901886, 0.00035181696875952184, -0.1, -0.2332201600074768, 0.51811283826828,
 -0.2762000858783722, 0.06211360543966293, 0.29983898997306824, -1.303462266921997, 2.0007927417755127, 0.49823325872421265,
  0.0003098883025813848, -0.0044272784143686295, 0.29982614517211914, 1.3034454584121704, 2.000779867172241, -0.498238742351532,
  0.0003156556049361825, 0.004448802210390568};

void SetJointStates(const sensor_msgs::JointState::ConstPtr &_js)
{
  static ros::Time startTime = ros::Time::now();
  {
    // for testing round trip time
    jointcommands.header.stamp = _js->header.stamp;

    // assign sinusoidal joint angle targets
    for (unsigned int i = 0; i < jointcommands.name.size(); i++)
      if( i == 7){
          jointcommands.position[i] = x1;
          x1 += 0.0001;
      }else if (i == 13){
          jointcommands.position[i] = x2;
          x2 += 0.0001;
      }else{
          jointcommands.position[i] = stand_prep_msg[i];
      }
    pub_joint_commands_.publish(jointcommands);
  }
  if (x1 >= 2.0 || x2 >= 2.0){
      exit(1);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_joint_command_test");

  ros::NodeHandle* rosnode = new ros::NodeHandle();

  // Waits for simulation time update.
  ros::Time last_ros_time_;
  bool wait = true;
  while (wait){
//    ROS_INFO("waiting");
    last_ros_time_ = ros::Time::now();
    if (last_ros_time_.toSec() > 0)
      wait = false;
  }
// back_lbz'1, 'back_mby'2, 'back_ubx'3, 'neck_ay'4, 'l_leg_uhz'5, 'l_leg_mhx'6, 'l_leg_lhy'7,
  // 'l_leg_kny'8, 'l_leg_uay'9, 'l_leg_lax'10, 'r_leg_uhz'11, 'r_leg_mhx'12, 'r_leg_lhy'13, 'r_leg_kny'14,
  // 'r_leg_uay'15, 'r_leg_lax'16, 'l_arm_usy'17, 'l_arm_shx'18, 'l_arm_ely'19, 'l_arm_elx'20, 'l_arm_uwy'21,
  // 'l_arm_mwx'22, 'r_arm_usy'23, 'r_arm_shx'24, 'r_arm_ely'25, 'r_arm_elx'26, 'r_arm_uwy'27, 'r_arm_mwx28
  // must match those inside AtlasPlugin
  jointcommands.name.push_back("atlas::back_lbz");//0
  jointcommands.name.push_back("atlas::back_mby");//1
  jointcommands.name.push_back("atlas::back_ubx");//2
  jointcommands.name.push_back("atlas::neck_ay");//3
  jointcommands.name.push_back("atlas::l_leg_uhz");//4
  jointcommands.name.push_back("atlas::l_leg_mhx");//5
  jointcommands.name.push_back("atlas::l_leg_lhy");//6
  jointcommands.name.push_back("atlas::l_leg_kny");//7*
  jointcommands.name.push_back("atlas::l_leg_uay");//8
  jointcommands.name.push_back("atlas::l_leg_lax");//9
  jointcommands.name.push_back("atlas::r_leg_uhz");//10
  jointcommands.name.push_back("atlas::r_leg_mhx");//11
  jointcommands.name.push_back("atlas::r_leg_lhy");//12
  jointcommands.name.push_back("atlas::r_leg_kny");//13*
  jointcommands.name.push_back("atlas::r_leg_uay");//14
  jointcommands.name.push_back("atlas::r_leg_lax");//15
  jointcommands.name.push_back("atlas::l_arm_usy");//16
  jointcommands.name.push_back("atlas::l_arm_shx");//17
  jointcommands.name.push_back("atlas::l_arm_ely");//18
  jointcommands.name.push_back("atlas::l_arm_elx");//19
  jointcommands.name.push_back("atlas::l_arm_uwy");//20
  jointcommands.name.push_back("atlas::l_arm_mwx");//21
  jointcommands.name.push_back("atlas::r_arm_usy");//22
  jointcommands.name.push_back("atlas::r_arm_shx");//23
  jointcommands.name.push_back("atlas::r_arm_ely");//24
  jointcommands.name.push_back("atlas::r_arm_elx");//25
  jointcommands.name.push_back("atlas::r_arm_uwy");//26
  jointcommands.name.push_back("atlas::r_arm_mwx");//27

  unsigned int n = jointcommands.name.size();
  jointcommands.position.resize(n);
  jointcommands.velocity.resize(n);
  jointcommands.effort.resize(n);
  jointcommands.kp_position.resize(n);
  jointcommands.ki_position.resize(n);
  jointcommands.kd_position.resize(n);
  jointcommands.kp_velocity.resize(n);
  jointcommands.i_effort_min.resize(n);
  jointcommands.i_effort_max.resize(n);

  for (unsigned int i = 0; i < n; i++)
  {
    std::vector<std::string> pieces;
    boost::split(pieces, jointcommands.name[i], boost::is_any_of(":"));

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p",
      jointcommands.kp_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i",
      jointcommands.ki_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d",
      jointcommands.kd_position[i]);

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
      jointcommands.i_effort_min[i]);
    jointcommands.i_effort_min[i] = -jointcommands.i_effort_min[i];

    rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
      jointcommands.i_effort_max[i]);

    jointcommands.velocity[i]     = 0;
    jointcommands.effort[i]       = 0;
    jointcommands.kp_velocity[i]  = 0;
  }

  // ros topic subscriptions
  ros::SubscribeOptions jointStatesSo =
    ros::SubscribeOptions::create<sensor_msgs::JointState>(
    "/atlas/joint_states", 1, SetJointStates,
    ros::VoidPtr(), rosnode->getCallbackQueue());

  // Because TCP causes bursty communication with high jitter,
  // declare a preference on UDP connections for receiving
  // joint states, which we want to get at a high rate.
  // Note that we'll still accept TCP connections for this topic
  // (e.g., from rospy nodes, which don't support UDP);
  // we just prefer UDP.
  jointStatesSo.transport_hints = ros::TransportHints().unreliable();

  ros::Subscriber subJointStates = rosnode->subscribe(jointStatesSo);
  // ros::Subscriber subJointStates =
  //   rosnode->subscribe("/atlas/joint_states", 1000, SetJointStates);

  pub_joint_commands_ =
    rosnode->advertise<osrf_msgs::JointCommands>(
    "/atlas/joint_commands", 1, true);

  ros::spin();

  return 0;
}
