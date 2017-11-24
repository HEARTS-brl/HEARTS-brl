#!/usr/bin/env python

### This is a script for testing movements with Tiago's arm
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

###Publishers




####Subscribers - listen for location to move arm to


destination = geometry_msgs.msg.Pose()

def callback(data):
    destination = data.position.z
    if destination != 0:
    	print"DESTINATION RECEIVED"
    	#rospy.loginfo("Destination Received")
    	pass

    else:
    	print"DESTINATION NOT RECEIVED"
    	#rospy.loginfo("No Destination Received")

rospy.Subscriber('/arm_destination_test', geometry_msgs.msg.Pose, callback)


print "============ Starting tutorial setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)


robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group = moveit_commander.MoveGroupCommander("arm_torso")

display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)

print "============ Waiting for RVIZ..."
rospy.sleep(10)
print "============ Starting tutorial "

print "============ Reference frame: %s" % group.get_planning_frame()

print "============ End effector: %s" % group.get_end_effector_link()

print "============ Robot Groups:"
print robot.get_group_names()

print "============ Printing robot state"
print robot.get_current_state()
print "============"


print "============ Generating plan 1"
pose_target = geometry_msgs.msg.Pose() 
q = quaternion_from_euler(-0.011, 1.57, 0.037) ## conversion from roll, pitch, and yaw into quaternions
pose_target.orientation.x = q[0]
pose_target.orientation.y = q[1]
pose_target.orientation.z = q[2]
pose_target.orientation.w = q[3]
pose_target.position.x = 0.7
pose_target.position.y = -0.3
pose_target.position.z = 0.8
group.set_pose_target(pose_target)
#group.setStartStateToCurrentState()
plan1 = group.plan()


'''
### start pose
start_pose = geometry_msgs.msg.Pose()
q = quaternion_from_euler(-0.011, 1.57, 0.037) ## conversion from roll, pitch, and yaw into quaternions
start_pose.orientation.x = q[0]
start_pose.orientation.y = q[1]
start_pose.orientation.z = q[2]
start_pose.orientation.w = q[3]
start_pose.position.x = 0.7
start_pose.position.y = -0.3
start_pose.position.z = 0.8

waypoints = []

#start with start_pose
waypoints.append(copy.deepcopy(start_pose))

# start with the current pose
#waypoints.append(group.get_current_pose().pose)

# first orient gripper and move forward (+x)
wpose = geometry_msgs.msg.Pose()
wpose.orientation.w = 1.0
wpose.position.x = waypoints[0].position.x + 0.1
wpose.position.y = waypoints[0].position.y
wpose.position.z = waypoints[0].position.z
waypoints.append(copy.deepcopy(wpose))

# second move down
wpose.position.z -= 0.0
waypoints.append(copy.deepcopy(wpose))

# third move to the side
wpose.position.y += 0.05
waypoints.append(copy.deepcopy(wpose))

#fourth move twist
wpose.orientation.w += 0.5
waypoints.append(copy.deepcopy(wpose))

#fourth move twist soem more
wpose.orientation.w -= 0.7
waypoints.append(copy.deepcopy(wpose))

(plan3, fraction) = group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold

print "============ Waiting while RVIZ displays plan3..."
rospy.sleep(5)
'''


#print "============ Waiting while RVIZ displays plan1..."
#rospy.sleep(5)

print "============ Visualizing plan1"
display_trajectory = moveit_msgs.msg.DisplayTrajectory()

display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan1)
#display_trajectory.trajectory.append(plan3)
display_trajectory_publisher.publish(display_trajectory)

print "test"
print "==========asdasdasd == Waiting while plan1 is visualized (again)..."
#rospy.sleep(5)
print "About to move!"

#group.move()

group.go(wait=True)

print"Done moving"

rospy.spin()
print"You spin me right round"

'''
if __name__ == '__main__':
	rospy.init_node('know_my_home', anonymous=True)
	rospy.loginfo("know my home controller has started")
	controller = Controller()
	rospy.spin()
'''	