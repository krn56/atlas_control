#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>

ros::Publisher pubJointCommands;
osrf_msgs::JointCommands jointCommands;

static float x1 = 0, x2 = 0;

void setJointStatesCb(const sensor_msgs::JointState::ConstPtr & js){
    jointCommands.header.stamp = js->header.stamp;
    jointCommands.header.frame_id = "atlas::pelvis";
    // Moving legs upward
    for (unsigned int i = 0; i < jointCommands.name.size(); i++){

        // pose_1 leg upwards values
        if( i == 7){
            jointCommands.position[i] = 1.9;
        }else if (i == 13){
            jointCommands.position[i] = 1.9;
        }

        // pose_2 values for arm movement
        else if (i == 17){
            jointCommands.position[i] = x1;
            x1 += 0.001;
        }else if (i == 23){
            jointCommands.position[i] = x2;
            x2 -= 0.001;
        }

        // values to keep legs balanced
        else if (i == 5){
            jointCommands.position[i] = -0.8;
            jointCommands.effort[i] = 200;
        }else if (i == 11){
            jointCommands.position[i] = 0.8;
            jointCommands.effort[i] = -200;
        }else if (i == 4){
            jointCommands.position[i] = -1.2;
            jointCommands.effort[i] = -200;
        }else if(i == 10){
            jointCommands.position[i] = 1.2;
            jointCommands.effort[i] = 200;
        }else{
            jointCommands.position[i] = 0.0;
        }

        if(x1 >= 1.39 || x2 <= -1.39){
            exit(1);
        }

    pubJointCommands.publish(jointCommands);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "initialPose");
    ros::NodeHandle* nh = new ros::NodeHandle();

    // wait for the simulation time update
    ros::Time last_ros_time;
    bool wait = true;
    while(wait){
        last_ros_time = ros::Time::now();
        if(last_ros_time.toSec() > 0)
            wait = false;
    }

    jointCommands.name.push_back("atlas::back_lbz");//0
    jointCommands.name.push_back("atlas::back_mby");//1
    jointCommands.name.push_back("atlas::back_ubx");//2
    jointCommands.name.push_back("atlas::neck_ay");//3
    jointCommands.name.push_back("atlas::l_leg_uhz");//4
    jointCommands.name.push_back("atlas::l_leg_mhx");//5
    jointCommands.name.push_back("atlas::l_leg_lhy");//6
    jointCommands.name.push_back("atlas::l_leg_kny");//7*
    jointCommands.name.push_back("atlas::l_leg_uay");//8
    jointCommands.name.push_back("atlas::l_leg_lax");//9
    jointCommands.name.push_back("atlas::r_leg_uhz");//10
    jointCommands.name.push_back("atlas::r_leg_mhx");//11
    jointCommands.name.push_back("atlas::r_leg_lhy");//12
    jointCommands.name.push_back("atlas::r_leg_kny");//13*
    jointCommands.name.push_back("atlas::r_leg_uay");//14
    jointCommands.name.push_back("atlas::r_leg_lax");//15
    jointCommands.name.push_back("atlas::l_arm_usy");//16
    jointCommands.name.push_back("atlas::l_arm_shx");//17
    jointCommands.name.push_back("atlas::l_arm_ely");//18
    jointCommands.name.push_back("atlas::l_arm_elx");//19
    jointCommands.name.push_back("atlas::l_arm_uwy");//20
    jointCommands.name.push_back("atlas::l_arm_mwx");//21
    jointCommands.name.push_back("atlas::r_arm_usy");//22
    jointCommands.name.push_back("atlas::r_arm_shx");//23
    jointCommands.name.push_back("atlas::r_arm_ely");//24
    jointCommands.name.push_back("atlas::r_arm_elx");//25
    jointCommands.name.push_back("atlas::r_arm_uwy");//26
    jointCommands.name.push_back("atlas::r_arm_mwx");//27

    unsigned int n = jointCommands.name.size();
    jointCommands.position.resize(n);
    jointCommands.velocity.resize(n);
    jointCommands.effort.resize(n);
    jointCommands.kp_position.resize(n);
    jointCommands.ki_position.resize(n);
    jointCommands.kd_position.resize(n);
    jointCommands.kp_velocity.resize(n);
    jointCommands.i_effort_min.resize(n);
    jointCommands.i_effort_max.resize(n);

    for (unsigned int i = 0; i < n; i++)
    {
      std::vector<std::string> pieces;
      boost::split(pieces, jointCommands.name[i], boost::is_any_of(":"));

      nh->getParam("atlas_controller/gains/" + pieces[2] + "/p",
        jointCommands.kp_position[i]);

      nh->getParam("atlas_controller/gains/" + pieces[2] + "/i",
        jointCommands.ki_position[i]);

      nh->getParam("atlas_controller/gains/" + pieces[2] + "/d",
        jointCommands.kd_position[i]);

      nh->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
        jointCommands.i_effort_min[i]);
      jointCommands.i_effort_min[i] = -jointCommands.i_effort_min[i];

      nh->getParam("atlas_controller/gains/" + pieces[2] + "/i_clamp",
        jointCommands.i_effort_max[i]);

      jointCommands.velocity[i]     = 0;
      jointCommands.effort[i]       = 0;
      jointCommands.kp_velocity[i]  = 0;
    }

    // ros topic subscriber
    ros::Subscriber jointStatesSub =  nh->subscribe("/atlas/joint_states", 1, setJointStatesCb);

    // ros topic pulisher
    pubJointCommands = nh->advertise<osrf_msgs::JointCommands>("/atlas/joint_commands", 1, true);

    ros::spin();

    return 0;
}
