#!/usr/bin/env python

import roslib; roslib.load_manifest('joint_animation')
import rospy, math, time

from sensor_msgs.msg import JointState

def jointTrajectoryCommand():
	# Initialize the node
	rospy.init_node('reach_joint_state')

	print rospy.get_rostime().to_sec()
	while rospy.get_rostime().to_sec() == 0.0:
		time.sleep(0.1)
		print rospy.get_rostime().to_sec()

	pub = rospy.Publisher('/atlas/joint_states', JointState, queue_size=10)
	jt = JointState()

	jt.header.stamp = rospy.Time.now()
	jt.header.frame_id = "atlas::pelvis"

	jt.name.append("atlas::back_lbz")
	jt.name.append("atlas::back_mby")
	jt.name.append("atlas::back_ubx")
	jt.name.append("atlas::neck_ay")
	jt.name.append("atlas::l_leg_uhz")
	jt.name.append("atlas::l_leg_mhx")
	jt.name.append("atlas::l_leg_lhy")
	jt.name.append("atlas::l_leg_kny")
	jt.name.append("atlas::l_leg_uay")
	jt.name.append("atlas::l_leg_lax")
	jt.name.append("atlas::r_leg_lax")
	jt.name.append("atlas::r_leg_uay")
	jt.name.append("atlas::r_leg_kny")
	jt.name.append("atlas::r_leg_lhy")
	jt.name.append("atlas::r_leg_mhx")
	jt.name.append("atlas::r_leg_uhz")
	jt.name.append("atlas::l_arm_elx")
	jt.name.append("atlas::l_arm_ely")
	jt.name.append("atlas::l_arm_mwx")
	jt.name.append("atlas::l_arm_shx")
	jt.name.append("atlas::l_arm_usy")
	jt.name.append("atlas::l_arm_uwy")
	jt.name.append("atlas::r_arm_elx")
	jt.name.append("atlas::r_arm_ely")
	jt.name.append("atlas::r_arm_mwx")
	jt.name.append("atlas::r_arm_shx")
	jt.name.append("atlas::r_arm_usy")
	jt.name.append("atlas::r_arm_uwy")

	# n = 1500
	# dt = 0.01
	# rps = 0.05

	# for i in range(n):
	# 	p = JointTrajectoryPoint()
	# 	theta = rps * 2.0 * math.pi * i * dt 
	# 	# x1 = -0.5 * math.sin(2 * theta)
	# 	# x2 =  0.5 * math.sin(1 * theta)
	# 	x1 = -i * 0.001
	# 	x2 = -i * 0.001
	x1 = 10
	x2 = 10	
	jt.position.append(x1)
	jt.position.append(x2)
	jt.position.append(x2)
	jt.position.append(x2)
	jt.position.append(x2)
	jt.position.append(x2)
	jt.position.append(x1)
	jt.position.append(x2)
	jt.position.append(x1)
	jt.position.append(x1)
	jt.position.append(x2)
	jt.position.append(x1)
	jt.position.append(x2)
	jt.position.append(x1)
	jt.position.append(x1)
	jt.position.append(x2)
	jt.position.append(x2)
	jt.position.append(x1)
	jt.position.append(x1)
	jt.position.append(x1)
	jt.position.append(x2)
	jt.position.append(x2)
	jt.position.append(x2)
	jt.position.append(x1)
	jt.position.append(x1)
	jt.position.append(x2)
	jt.position.append(x1)
	jt.position.append(x1)

	# jtt.points.ppend(p) 

	# set duration
	# jt.points[i].time_from_start = rospy.Duration.from_sec(dt)
	# rospy.loginfo("test: angles[%d][%f, %f]", i, x1, x2)

	pub.publish(jt)
	rospy.spin()

if __name__ == "__main__":
	try:
		jointTrajectoryCommand()
	except rospy.ROSInterruptException: pass
