#!/usr/bin/env python

## mavros
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
## Tool

#topic --> PoseStamped()
custom_pose = PoseStamped() #[Header, geometry_msgs/Pose]

from geometry_msgs.msg import Pose

#----------------------#

Pose_msg = Pose()
'''
Pose_msg.position    #geometry_msgs/Point --> [float64, float64, float64]
Pose_msg.orientation #geometry_msgs/Quaternion --> [float64, float64, float64, float64]
'''
Pose_msg.position = [0.0, 0.0, 0.0]
Pose_msg.orientation = [0.0, 0.0, 0.0, 1.0]


custom_pose.pose = Pose_msg

#---------------------#

'''
Caculation

image+IMU --> pose + position

'''

class ROS_FOR_EXAMPLE():

    def __init__(self):

        self.pose = []

        print("hello")

    def publish_to_ros(self, pose, state):

        self.pose = pose     #[float64, float64, float64]
        now_state = state   #[float64, float64, float64, float64]

        #Pose()
        now_pose_msg = Pose()
        now_pose_msg.position = now_pose
        now_pose_msg.orientarion = now_state

        #PoseStamped()
        custom_posestamped = PoseStamped()
        custom_posestamped.pose = now_pose_msg

        rospy.publish("/mavros/posestamped", PoseStamped(), custom_posestamped)
##---------------------------

def publish_to_ros(pose, state):

    now_pose = pose     #[float64, float64, float64]
    now_state = state   #[float64, float64, float64, float64]

    #Pose()
    now_pose_msg = Pose()
    now_pose_msg.position = now_pose
    now_pose_msg.orientarion = now_state

    #PoseStamped()
    custom_posestamped = PoseStamped()
    custom_posestamped.pose = now_pose_msg

    rospy.publish("/mavros/posestamped", PoseStamped(), custom_posestamped)


pose_ex = [x, y, z]
state_ex = [x, y, z, w]

publish_to_ros(pose_ex, state_ex)

#---------------------#

import numpy as asd
import rospy
import tf
import copy
import cv2
import time
import heapq
from math import *
from pandas import DataFrame
from datetime import datetime
## File
from AOA_v1 import AOA
from dop import cost_function
from LeastQ_v1 import least_square
from hsv import hsv

AOA = AOA()
cal_dop = cost_function()
LeastQ = least_square()
current_state = State()
hsv = hsv()

enu_pos = []
azimuth = []
ob_list = []
est_position = []
est_vector = []
next_position_list = []
RMSE_list = []
error = []
landing_point = []

class aoa_info(object):
    def __init__(self):
        self.imu_msg = Imu()
        self.gps_msg = Odometry()
        self.current_state = State()

        self.gps_pose = [0,0,0]
        self.ned_pose = [0,0,0]
        self.ob_pose = [0,0,0]
        self.imu_x = 0
        self.roll, self.pitch, self.yaw = 0,0,0
        self.quat = [0,0,0,0]
        self.u, self.v = 0, 0
        self.uu, self.vv = 50, 50
        self.a, self.b = 0, 0
        self.P_img_x, self.P_img_y, self.P_img_z = 0, 0, 0
        self.angle_a_w = 0
        self.angle_e_w = 0
        self.angle_a = [0, 0]
        self.angle_e = [0, 0]
        self.est_position = []
        self.est_x, self.est_y, self.est_z = 0, 0, 0
        self.ob_point = [0, 0, 0]
        self.next_waypoint = [0, 0]
        self.next_heading = 0
        self.next_waypoint_2 = [0, 0]
        self.next_heading_2 = 0
        self.est_n, self.est_e, self.est_d = 0, 0, 0
        self.vector_n, self.vector_e, self.vector_d = 0, 0, 0
        self.last_req = rospy.Time.now()

        rospy.wait_for_service("/iris_fpv_cam_0/mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy("/iris_fpv_cam_0/mavros/set_mode", SetMode)
        self.offb_set_mode = SetModeRequest()
        rospy.wait_for_service("/iris_fpv_cam_0/mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("iris_fpv_cam_0/mavros/cmd/arming", CommandBool)
        self.arm_cmd = CommandBoolRequest()

        rospy.Subscriber("/iris_fpv_cam_0/mavros/state", State, self.state_cb)
        rospy.Subscriber("/iris_fpv_cam_0/mavros/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/iris_fpv_cam_0/mavros/global_position/local", Odometry, self.gps_callback)
            
        self.pos_pub = rospy.Publisher("/iris_fpv_cam_0/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.pose = PoseStamped() # only send position
        #self.setpoint_pub = rospy.Publisher("/iris_fpv_cam_0/mavros/setpoint_raw/local", PositionTarget, queue_size=1) 
        #self.setpoint = PositionTarget()

    def state_cb(self, msg):
        self.current_state = msg

    def gps_callback(self, msg):
        self.gps_msg = msg
        self.gps_pose[0] = msg.pose.pose.position.x
        self.gps_pose[1] = msg.pose.pose.position.y
        self.gps_pose[2] = msg.pose.pose.position.z

        self.ned_pose[0], self.ned_pose[1], self.ned_pose[2] = self.gps_pose[1], self.gps_pose[0], -self.gps_pose[2]
        #print(self.ned_pose[0], self.ned_pose[1], self.ned_pose[2])

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.quat[0] = msg.orientation.w
        self.quat[1] = msg.orientation.x
        self.quat[2] = msg.orientation.y
        self.quat[3] = msg.orientation.z

        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

    def cal_aoa_info(self):

        if [self.u, self.v]!=[None, None] and [self.u, self.v]!=[0, 0]:

            # position_vector
            size_u = 640
            size_v = 360
            u_0 = size_u/2
            v_0 = size_v/2
            # focal length
            f = 277.191356 
            #f = 554.382712
            #f = 1108.765424
        
            self.P_img_x = u_0 - self.u 
            self.P_img_y = v_0 - self.v
            self.P_img_z = f
            
            # observstion point
            self.ob_point[0] = self.ned_pose[0]
            self.ob_point[1] = self.ned_pose[1]
            self.ob_point[2] = self.ned_pose[2]

            ###################### AOA ######################
            self.angle_a_w, self.angle_e_w, self.angle_a, self.angle_e, self.est_n, self.est_e, self.est_d, self.vector_n, self.vector_e, self.vector_d = AOA.AOA_v0(self.ned_pose[0], self.ned_pose[1], self.ned_pose[2], self.roll, self.pitch, self.yaw, self.P_img_x, self.P_img_y, self.P_img_z)
            print("---------------AOA----------------")
            #print('P_img =',self.P_img_x, self.P_img_y, self.P_img_z)
            print('azimuth = ',self.angle_a_w)
            print('elevation = ',self.angle_e_w)
            print('Target_position_world (ned) = ', self.est_n, self.est_e, self.est_d)

            ############## collect data for observation point #####################
            est_position.append([self.est_n, self.est_e, self.est_d])
            est_vector.append([self.vector_n, self.vector_e, self.vector_d])
            azimuth.append(self.angle_a_w)
            azimuth_num = len(azimuth)
            ob_list.append([self.ob_point[0],self.ob_point[1],self.ob_point[2]])
            #print('azimuth_num')
            #print(azimuth_num)
            # print('azimuth = ')
            # print(azimuth)
            # print('ob_list = ')
            # print(ob_list)
            # print('est_position = ')
            # print(est_position)

    def mission(self):

        if len(est_position)==1 :
            ###################### path planning ######################
            value = []
            D_value = []
   
            next_position_list =  cal_dop.next_position(azimuth[0], est_position[0])
            # print("next_position_list =", next_position_list)

            ############# Find observaiton point #############
            for i in range (20):
                a = [next_position_list[i][0], next_position_list[i][1], next_position_list[i][2]]
                GDOP, D = cal_dop.calculate_dop(ob_list[0], a, est_position[0])
                #GDOP = cal_dop.calculate_dop(ob_list[0], a, est)
                value.append(GDOP)
                D_value.append(D)
            # print("GDOP_list = ")
            # print(value)
            # print("determinant_list = ")
            # print(D_value)
            
            min_dop = min(value)
            index = value.index(min_dop)
            print('index =', index)

            max_d = heapq.nlargest(2, D_value)
            index_d_1 = D_value.index(max_d[0])
            index_d_2 = D_value.index(max_d[1])
            # print('max_d =')
            # print(max_d)
            # print('index_d_1, index_d_2 =')
            # print(index_d_1, index_d_2)
            ##################################################

            self.next_waypoint = next_position_list[index]
            self.next_heading = np.arctan2(est_position[0][0]-self.next_waypoint[0], est_position[0][1]-self.next_waypoint[1])
            print('Next observation point =', self.next_waypoint)

    def waypoint_command(self, w1, h1):

        if (self.current_state.mode != "OFFBOARD" and (rospy.Time.now()-self.last_req) > rospy.Duration(0.5)):
            offb_set_mode = SetModeRequest()
            offb_set_mode.custom_mode = 'OFFBOARD'
            if(self.set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            self.last_req = rospy.Time.now()
        else:
            self.pose.pose.position.x = w1[1]  #e
            self.pose.pose.position.y = w1[0]  #n
            self.pose.pose.position.z = 10     #u
            #quat = tf.transformations.quaternion_from_euler(0, 0, np.pi/2)  #East
            quat = tf.transformations.quaternion_from_euler(0, 0, h1)
            self.pose.pose.orientation.x = quat[0]
            self.pose.pose.orientation.y = quat[1]
            self.pose.pose.orientation.z = quat[2]
            self.pose.pose.orientation.w = quat[3]
            self.pos_pub.publish(self.pose)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
  
    def ENU2NED(self,x, y ,z):
        R = [[0, 1, 0],[1, 0, 0],[0, 0, -1]]
        q = [x, y, z]
        ned = np.matmul(R,q)
        a = ned[0]
        b = ned[1]
        c = ned[2]
        return a, b, c

    def iteration(self, event):

        self.u, self.v = hsv.value_callback()

        if len(azimuth)==0 :

            self.cal_aoa_info()
            RMSE_0 = np.sqrt(np.square(est_position[0][1]-10)+np.square(est_position[0][0]-50))
            RMSE_list.append(RMSE_0)
            print('RMSE_0 =', RMSE_0)

            self.mission()

        elif len(azimuth)==1 :

            w1 = self.next_waypoint
            h1 = self.next_heading
            w2 = self.next_waypoint_2
            h2 = self.next_heading_2
            self.waypoint_command(w1, h1)

            if np.sqrt(np.square(self.gps_pose[2]-10)+np.square(self.gps_pose[0]- w1[1])+np.square(self.gps_pose[1]- w1[0]))<= 0.5:
                self.cal_aoa_info()

        elif len(azimuth)==2 :

            ############# 1 : Least square ###############
            Est_n, Est_e, Est_d = LeastQ.LeastQ(ob_list,azimuth) #ned

            landing_point = [Est_n, Est_e, Est_d]
            print("landing_point(NED) =", Est_n, Est_e, Est_d)
            RMSE_1 = np.sqrt(np.square(Est_e-10)+np.square(Est_n-50))
            print('RMSE_1 =', RMSE_1)
            RMSE_list.append(RMSE_1)

            ############ 2 : two vector insective ###############
            # line equation : (a1,a2)observation point (v1,v2)observation vector
            # y = (v2/v1)*x+(a2-(v2/v1)*a1)
            # k1 = (est_vector[0][0]/est_vector[0][1])
            # k2 = (est_vector[1][0]/est_vector[1][1])
            # b1 = (ob_list[0][0]-(est_vector[1][0]/est_vector[1][1])*ob_list[0][1])
            # b2 = (ob_list[1][0]-(est_vector[1][0]/est_vector[1][1])*ob_list[1][1])
            # # y1 = k1*x1 + b1
            # # y2 = k2*x2 + b2
            # Est_e = ((b2-b1)/(k1-k2))
            # Est_n = k1*Est_e + b1
    
            # landing_point.append([Est_n, Est_e])
            # print('landing_point (NED:50,10)= ',landing_point)
            # RMSE_1 = np.sqrt(np.square(Est_e-10)+np.square(Est_n-50))
            # print('RMSE_1 =', RMSE_1)
            # RMSE_list.append(RMSE_1)
            #print(landing_point[0][0][0])
            ############################################################

            self.waypoint_command(landing_point, 0)


        enu_pos.append([self.gps_pose[0], self.gps_pose[1], self.gps_pose[2]])
       
if __name__ == '__main__':
    rospy.init_node('aoa_info_drone', anonymous=True)
    dt = 1.0/10
    #dt = 5
    pathplan_run = aoa_info()
    pathplan_run.mission()
    rospy.Timer(rospy.Duration(dt), pathplan_run.iteration)
    rospy.spin()
    
    df = DataFrame({'enu_pos': enu_pos})
    df.to_excel('drone_position.xlsx', sheet_name='sheet1', index=False)
    dd = DataFrame({'azimuth':azimuth,'ob_list': ob_list,'est_position': est_position})
    dd.to_excel('drone_result.xlsx', sheet_name='sheet1', index=False)


    # ----------------------------------------
    # ----------------- ------------------------
    # --------------        --------------------
    # -------------          -------------------
    # --------------        --------------------
    # ------------------ -----------------------
    # ------------------ -----------------------
    # ----------------      --------------------
    # -------------- -     - -------------------
    # ------------- --     -- -----------------
    # ------------ ---     --- ------------------
    # ----------------     --------------------
    # ---------------  ---  -------------------
    # ---------------  ---  --------------------
    # ---------------  ---  --------------------
    # ---------------  ---  --------------------
    # -------------    ---    ------------------
    # ------------------------------------------



