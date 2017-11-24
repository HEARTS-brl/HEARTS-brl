#!/usr/bin/env python
import rospy
from control_msgs.msg import PointHeadActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
#from pal_startup_msgs.srv import StartupStart, StartupStop
from pal_common_msgs.msg import DisableGoal, DisableAction
import actionlib


class NavigationCameraMgr:

    def __init__(self):
        self.previous_status = None

        self.pub_head_topic = rospy.Publisher(
            '/head_controller/point_head_action/goal',
            PointHeadActionGoal, queue_size=1)

        self.torso_cmd = rospy.Publisher('/torso_controller/command',
                                         JointTrajectory, queue_size=1)

        self.head_mgr_client = actionlib.SimpleActionClient(
            '/pal_head_manager/disable', DisableAction)

        rospy.loginfo("monitoring move_base goal "
                      "status and moving the camera accordingly")

    def head_mgr_as(self, command):

        if(not self.head_mgr_client.wait_for_server(
                timeout=rospy.Duration(2.0))):
            rospy.logwarn("failed to connect to head manager node")
            return

        if (command == "enable"):
            rospy.loginfo("enabling head manager.")
            self.head_mgr_client.cancel_goal()

        elif (command == "disable"):
            action = DisableGoal()
            rospy.loginfo("disabling head manager")
            action.duration = 0.0
            self.head_mgr_client.send_goal(action)

    def head_mgr_service(self, command):
        if (command == "start"):
            try:
                rospy.wait_for_service('/pal_startup_control/start', 2)
            except rospy.ROSException and rospy.ServiceException as e:
                rospy.logerr(
                    'Could not reach pal_startup_control/start : %s', e.message)

            pal_start = rospy.ServiceProxy(
                '/pal_startup_control/start', StartupStart)
            try:
                rospy.loginfo("enabling head_manager.")
                pal_start("head_manager", "")
            except rospy.ROSException and rospy.ServiceException as e:
                rospy.logerr('Could not start head_manager: %s', e.message)

        elif (command == "stop"):
            try:
                rospy.wait_for_service('/pal_startup_control/stop', 2)
            except rospy.ROSException and rospy.ServiceException as e:
                rospy.logerr(
                    'Could not reach pal_startup_control/stop : %s', e.message)
            pal_stop = rospy.ServiceProxy(
                '/pal_startup_control/stop', StartupStop)
            try:
                rospy.loginfo("disabling head_manager.")
                pal_stop("head_manager")
            except rospy.ROSException and rospy.ServiceException as e:
                rospy.logerr('Could not stop head_manager: %s', e.message)

    def elevate_torso(self):
        rospy.loginfo("Moving torso up")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.20]
        jtp.time_from_start = rospy.Duration(2.5)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)

    def look_down(self):
        phag = PointHeadActionGoal()
        phag.header.frame_id = "/base_link"
        phag.goal.max_velocity = 1.0
        phag.goal.min_duration = rospy.Duration(0.2)

        phag.goal.target.header.frame_id = "/base_link"
        phag.goal.target.point.x = 0.9

        phag.goal.pointing_axis.x = 1.0
        phag.goal.pointing_frame = "/head_2_link"

        self.pub_head_topic.publish(phag)

    def look_up(self):
        phag = PointHeadActionGoal()
        phag.header.frame_id = "/base_link"
        phag.goal.max_velocity = 1.0
        phag.goal.min_duration = rospy.Duration(0.2)

        phag.goal.target.header.frame_id = "/base_link"
        phag.goal.target.point.x = 0.9
        phag.goal.target.point.z = 1.0

        phag.goal.pointing_axis.x = 1.0
        phag.goal.pointing_frame = "/head_2_link"

        self.pub_head_topic.publish(phag)
