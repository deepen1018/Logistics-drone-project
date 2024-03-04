#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import PositionTarget
import tf
import numpy as np
import time
from nav_msgs.msg import Odometry
current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


class offb:

    def __init__(self):
        
        self.gps_msg=Odometry()
        self.gps_pose=[0,0,0]

    def gps_callback(self, msg):
            self.gps_msg = msg
            self.gps_pose[0] = msg.pose.pose.position.x
            self.gps_pose[1] = msg.pose.pose.position.y
            self.gps_pose[2] = msg.pose.pose.position.z






    def offboard(self):

        # rospy.init_node("offb_node_py")

        state_sub = rospy.Subscriber("/iris_fpv_cam_0/mavros/state", State, callback =state_cb)
        rospy.wait_for_service("/iris_fpv_cam_0/mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("iris_fpv_cam_0/mavros/cmd/arming", CommandBool)    
        rospy.wait_for_service("/iris_fpv_cam_0/mavros/set_mode")
        set_mode_client = rospy.ServiceProxy("iris_fpv_cam_0/mavros/set_mode", SetMode)
        rospy.Subscriber("/iris_fpv_cam_0/mavros/global_position/local", Odometry, self.gps_callback)

        setpoint_pub = rospy.Publisher("/iris_fpv_cam_0/mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        local_pos_pub = rospy.Publisher("/iris_fpv_cam_0/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
        # self.gps_pose[0] = msg.pose.pose.position.x
        # self.gps_pose[1] = msg.pose.pose.position.y
        # self.gps_pose[2] = msg.pose.pose.position.z

        # Setpoint publishing MUST be faster than 2Hz
        rate = rospy.Rate(20)

        # Wait for Flight Controller connection
        while(not rospy.is_shutdown() and not current_state.connected):
            rate.sleep()

        pose = PoseStamped()

        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 10
        
        quat = tf.transformations.quaternion_from_euler(0, 0, np.pi/2)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        # Send a few setpoints before starting
        for i in range(100):   
            if(rospy.is_shutdown()):
                break

            local_pos_pub.publish(pose)
            rate.sleep()

        offb_set_mode = SetModeRequest()
        offb_set_mode.custom_mode = 'OFFBOARD'

        arm_cmd = CommandBoolRequest()
        arm_cmd.value = True

        last_req = rospy.Time.now()

        while(not rospy.is_shutdown()):
            if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")

                    last_req = rospy.Time.now()

            local_pos_pub.publish(pose)
            if np.sqrt(np.square(self.gps_pose[2]-10)+np.square(self.gps_pose[0])+np.square(self.gps_pose[1]))<= 0.5:
                time.sleep(5)
                break


            rate.sleep()


