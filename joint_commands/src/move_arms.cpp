#include <math.h>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <iostream>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>

ros::Publisher pub_joint_commands_;
osrf_msgs::JointCommands jointcommands;

void SetJointStates(const sensor_msgs::JointState::ConstPtr &_js){
    static ros::Time startTime = ros::Time::now();
    {
        // for testing round trip time
        jointcommands.header.stamp = _js->header.stamp;

        // assign sinusoidal joint angle targets
        for (unsigned int i = 0; i < jointcommands.name.size(); i++){
            if(i == 7 || i == 13) {

                // jointcommands.position[i] = 3.2 * sin((ros::Time::now() - startTime).toSec());
                jointcommands.position[i] = 2.00;
                std::cout << "moved joint name - " << jointcommands.name[i] << " to :" << jointcommands.position[i] << std::endl;
            }else if(3 >= i || i >= 16){
                jointcommands.position[i] = 0;
                std::cout << "moved joint name - " << jointcommands.name[i] << " to :" << jointcommands.position[i] << std::endl;
            }else{
                jointcommands.position[i] = 0;
                std::cout << "moved joint name - " << jointcommands.name[i] << " to :" << jointcommands.position[i] << std::endl;
            }
        }

        pub_joint_commands_.publish(jointcommands);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "publish_joint_commands");

    ros::NodeHandle* rosnode = new ros::NodeHandle();

    // Waits for simulation time update
    ros::Time last_ros_time_;
    bool wait = true;
    while(wait){
        last_ros_time_ = ros::Time::now();
        if(last_ros_time_.toSec() > 0)
            wait = false;
    }


    // must match with AtlasPlugin
    jointcommands.name.push_back("atlas::back_lbz"); //0
    jointcommands.name.push_back("atlas::back_mby"); //1
    jointcommands.name.push_back("atlas::back_ubx"); //2
    jointcommands.name.push_back("atlas::neck_ay"); //3
    jointcommands.name.push_back("atlas::l_leg_uhz"); //4
    jointcommands.name.push_back("atlas::l_leg_mhx"); //5
    jointcommands.name.push_back("atlas::l_leg_lhy"); //6
    jointcommands.name.push_back("atlas::l_leg_kny"); //7
    jointcommands.name.push_back("atlas::l_leg_uay"); //8
    jointcommands.name.push_back("atlas::l_leg_lax"); //9
    jointcommands.name.push_back("atlas::r_leg_uhz"); //10
    jointcommands.name.push_back("atlas::r_leg_mhx"); //11
    jointcommands.name.push_back("atlas::r_leg_lhy"); //12
    jointcommands.name.push_back("atlas::r_leg_kny"); //13
    jointcommands.name.push_back("atlas::r_leg_uay"); //14
    jointcommands.name.push_back("atlas::r_leg_lax"); //15
    jointcommands.name.push_back("atlas::l_arm_usy"); //16
    jointcommands.name.push_back("atlas::l_arm_shx"); //17
    jointcommands.name.push_back("atlas::l_arm_ely"); //18
    jointcommands.name.push_back("atlas::l_arm_elx"); //19
    jointcommands.name.push_back("atlas::l_arm_uwy"); //20
    jointcommands.name.push_back("atlas::l_arm_mwx"); //21
    jointcommands.name.push_back("atlas::r_arm_usy"); //22
    jointcommands.name.push_back("atlas::r_arm_shx"); //23
    jointcommands.name.push_back("atlas::r_arm_ely"); //24
    jointcommands.name.push_back("atlas::r_arm_elx"); //25
    jointcommands.name.push_back("atlas::r_arm_uwy"); //26
    jointcommands.name.push_back("atlas::r_arm_mwx"); //27

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

    for (unsigned int i = 0; i < n; i++){
        std::vector<std::string> pieces;
        boost::split(pieces, jointcommands.name[i], boost::is_any_of(":"));

        rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/p", jointcommands.kp_position[i]);
        rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i", jointcommands.ki_position[i]);
        rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/d", jointcommands.kd_position[i]);
        rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", jointcommands.i_effort_min[i]);
        jointcommands.i_effort_min[i] = -jointcommands.i_effort_min[i];
        rosnode->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp", jointcommands.i_effort_max[i]);

        jointcommands.velocity[i]    = 0;
        jointcommands.effort[i]      = 0;
        jointcommands.kp_velocity[i] = 0;
    }

    // ros topic subscriptions
    ros::SubscribeOptions jointStatesSo = ros::SubscribeOptions::create<sensor_msgs::JointState>(
                "/atlas/joint_states", 1, SetJointStates, ros::VoidPtr(), rosnode->getCallbackQueue());

    jointStatesSo.transport_hints = ros::TransportHints().unreliable();

    ros::Subscriber subJointStates = rosnode->subscribe(jointStatesSo);

    pub_joint_commands_ = rosnode->advertise<osrf_msgs::JointCommands>("/atlas/joint_commands", 1 ,true);

    ros::spin();
    return 0;
}
