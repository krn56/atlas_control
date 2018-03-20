# include <ros/ros.h>
# include <math.h>
# include <trajectory_msgs/JointTrajectory.h>
# include <trajectory_msgs/JointTrajectoryPoint.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "move_arms_step5_node");

    // wait for simulation time update
    ros::Time last_ros_time;
    bool wait = true;
    while(wait){
//        ROS_INFO("waiting");
        last_ros_time = ros::Time::now();
        if (last_ros_time.toSec() > 0){
            wait = false;
        }
    }

    ros::NodeHandle nh;
    ros::Publisher traj_pub = nh.advertise <trajectory_msgs::JointTrajectory> ("/joint_trajectory", 10);

    trajectory_msgs::JointTrajectory jt;
    jt.header.stamp = ros::Time::now();
    jt.header.frame_id = "atlas::pelvis";

    jt.joint_names.push_back("atlas::back_lbz");//0
    jt.joint_names.push_back("atlas::back_mby");//1
    jt.joint_names.push_back("atlas::back_ubx");//2
    jt.joint_names.push_back("atlas::neck_ay");//3
    jt.joint_names.push_back("atlas::l_leg_uhz");//4
    jt.joint_names.push_back("atlas::l_leg_mhx");//5
    jt.joint_names.push_back("atlas::l_leg_lhy");//6
    jt.joint_names.push_back("atlas::l_leg_kny");//7
    jt.joint_names.push_back("atlas::l_leg_uay");//8
    jt.joint_names.push_back("atlas::l_leg_lax");//9
    jt.joint_names.push_back("atlas::r_leg_lax");//10
    jt.joint_names.push_back("atlas::r_leg_uay");//11
    jt.joint_names.push_back("atlas::r_leg_kny");//12
    jt.joint_names.push_back("atlas::r_leg_lhy");//13
    jt.joint_names.push_back("atlas::r_leg_mhx");//14
    jt.joint_names.push_back("atlas::r_leg_uhz");//15
    jt.joint_names.push_back("atlas::l_arm_elx");//16
    jt.joint_names.push_back("atlas::l_arm_ely");//17
    jt.joint_names.push_back("atlas::l_arm_mwx");//18
    jt.joint_names.push_back("atlas::l_arm_shx");//19*
    jt.joint_names.push_back("atlas::l_arm_usy");//20
    jt.joint_names.push_back("atlas::l_arm_uwy");//21
    jt.joint_names.push_back("atlas::r_arm_elx");//22
    jt.joint_names.push_back("atlas::r_arm_ely");//23
    jt.joint_names.push_back("atlas::r_arm_mwx");//24
    jt.joint_names.push_back("atlas::r_arm_shx");//25*
    jt.joint_names.push_back("atlas::r_arm_usy");//26
    jt.joint_names.push_back("atlas::r_arm_uwy");//27

    int n = 1200;
    float dt = 0.01;
    float rps = 0.05;
    static double i = 1.39;
   while(ros::ok()){
        trajectory_msgs::JointTrajectoryPoint p;
        double x1 = i;
        double x = 0;
//        double x1 = 2.0;
        p.positions.push_back(x);//0
        p.positions.push_back(x);//1
        p.positions.push_back(x);//2
        p.positions.push_back(x);//3
        p.positions.push_back(x);//4
        p.positions.push_back(x);//5
        p.positions.push_back(x);//6
        p.positions.push_back(2.0);//7
        p.positions.push_back(x);//8
        p.positions.push_back(x);//9
        p.positions.push_back(x);//10
        p.positions.push_back(x);//11
        p.positions.push_back(2.0);//12
        p.positions.push_back(x);//13
        p.positions.push_back(x);//14
        p.positions.push_back(x);//15
        p.positions.push_back(2.2);//16
        p.positions.push_back(x);//17
        p.positions.push_back(x);//18
        p.positions.push_back(x1);//19*
        p.positions.push_back(-1.3);//20
        p.positions.push_back(x);//21
        p.positions.push_back(-2.2);//22
        p.positions.push_back(x);//23
        p.positions.push_back(x);//24
        p.positions.push_back(-x1);//25*
        p.positions.push_back(-1.3);//26
        p.positions.push_back(x);//27

        jt.points.push_back(p);
        ROS_INFO("test: angles[%f][%f %f]",i,x,x1 );

        traj_pub.publish(jt);
        ros::spinOnce();
        i -= 0.0001;
        if (i == 0.4){
            exit(1);
        }
    }

    return 0;
}

