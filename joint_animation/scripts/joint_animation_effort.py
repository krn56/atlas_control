#!/usr/bin/env python

import roslib; roslib.load_manifest('joint_animation')
import rospy, math, time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def jointTrajectoryCommand():
	# Initialize the node
	rospy.init_node('joint_control')

	print rospy.get_rostime().to_sec()
	while rospy.get_rostime().to_sec() == 0.0:
		time.sleep(0.1)
		print rospy.get_rostime().to_sec()

	pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)
	jt = JointTrajectory()

	jt.header.stamp = rospy.Time.now()
	jt.header.frame_id = "atlas::pelvis"

	jt.joint_names.append("atlas::back_lbz")
	jt.joint_names.append("atlas::back_mby")
	jt.joint_names.append("atlas::back_ubx")
	jt.joint_names.append("atlas::neck_ay")
	jt.joint_names.append("atlas::l_leg_uhz")
	jt.joint_names.append("atlas::l_leg_mhx")
	jt.joint_names.append("atlas::l_leg_lhy")
	jt.joint_names.append("atlas::l_leg_kny")
	jt.joint_names.append("atlas::l_leg_uay")
	jt.joint_names.append("atlas::l_leg_lax")
	jt.joint_names.append("atlas::r_leg_lax")
	jt.joint_names.append("atlas::r_leg_uay")
	jt.joint_names.append("atlas::r_leg_kny")
	jt.joint_names.append("atlas::r_leg_lhy")
	jt.joint_names.append("atlas::r_leg_mhx")
	jt.joint_names.append("atlas::r_leg_uhz")
	jt.joint_names.append("atlas::l_arm_elx")
	jt.joint_names.append("atlas::l_arm_ely")
	jt.joint_names.append("atlas::l_arm_mwx")
	jt.joint_names.append("atlas::l_arm_shx")
	jt.joint_names.append("atlas::l_arm_usy")
	jt.joint_names.append("atlas::l_arm_uwy")
	jt.joint_names.append("atlas::r_arm_elx")
	jt.joint_names.append("atlas::r_arm_ely")
	jt.joint_names.append("atlas::r_arm_mwx")
	jt.joint_names.append("atlas::r_arm_shx")
	jt.joint_names.append("atlas::r_arm_usy")
	jt.joint_names.append("atlas::r_arm_uwy")

	n = 1500
	dt = 0.01
	rps = 0.05

	for i in range(n):
		p = JointTrajectoryPoint()
		theta = rps * 2.0 * math.pi * i * dt 
		x1 = 50 * abs(math.sin(2 * theta))
		# x2 =  0.5 * math.sin(1 * theta)
		# x1 = -0.001 * (500-i)
		x2 = i * 0.01
		x = 0.1
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x) # left leg hip abount z
		p.positions.append(x) # left leg hip about y
		p.positions.append(x) # left leg hip about x 
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)
		p.positions.append(x)

		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x) # left leg hip abount z
		p.effort.append(x) # left leg hip about y
		p.effort.append(x1) # left leg hip about x // important
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)
		p.effort.append(x)

		jt.points.append(p) 

		# set duration
		jt.points[i].time_from_start = rospy.Duration.from_sec(dt)
		rospy.loginfo("test: angles[%d][%f, %f]", i, x1, x2)

	pub.publish(jt)
	rospy.spin()

if __name__ == "__main__":
	try:
		jointTrajectoryCommand()
	except rospy.ROSInterruptException: pass
