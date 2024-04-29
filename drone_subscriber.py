import rospy
from sensor_msgs.msg import Image,CameraInfo
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point, PoseWithCovarianceStamped, PoseStamped
from std_srvs.srv import SetBool

import airsim
import math
import numpy as np

from airsim import YawMode

class AirsimSubscriber:

    current_pos: PoseStamped = PoseStamped() 
    isPrintGroundTruth: bool = False
    flyto_speed_limit = 1
    flyto_angular_limit = 0.2
    flyto_distance_threshold = 1
    diff_angle_threshold = 0.1
    angle_deadzone_threshold = 0.9

    def vel_cmd_body_frame_cb(self, twist: Twist):
        cmd_duration = 0.05
        yaw_mode = YawMode()
        yaw_mode.is_rate = True
        yaw_mode.yaw_or_rate = -math.degrees(twist.angular.z)
        client.moveByVelocityBodyFrameAsync(vx=twist.linear.x, 
                                   vy=-twist.linear.y, 
                                   vz=-twist.linear.z, 
                                   duration=cmd_duration,
                                   yaw_mode = yaw_mode)
        
        #rospy.loginfo("twist info linera:[%f,%f,%f] . angular %f", twist.linear.x,twist.linear.y,twist.linear.z, twist.angular.z)
        #, twist.angular.x, twist.angular.y, twist.angular.z)

    def pose_cb(self, pose_cov: PoseWithCovarianceStamped): # msg translator

        sub.current_pos.header = pose_cov.header
        sub.current_pos.pose = pose_cov.pose.pose
        publisher_pose.publish(sub.current_pos) # publish pose info to other node
        
        if self.isPrintGroundTruth:
            p = sub.current_pos.pose.position
            o = sub.current_pos.pose.orientation
            airsim_point = client.simGetGroundTruthEnvironment().position
            rospy.loginfo("estimate pose: [%f,%f,%f] orientation: [%f,%f,%f]",p.x,p.y,p.z, o.x, o.y, o.z)
            rospy.loginfo("airsim pose: [%f,%f,%f] ",airsim_point.x_val,airsim_point.y_val,airsim_point.z_val)
        
        

    def flyTo_cb(self, pose: PoseStamped): # publish a vel_cmd

        cmd: Twist = Twist()

        current_p = sub.current_pos.pose.position
        current_yaw_z = sub.current_pos.pose.orientation.z
        flyto_p = pose.pose.position
        flyto_o = pose.pose.orientation
        flyto_yaw = euler_from_quaternion([flyto_o.x,
                                            flyto_o.y,
                                            flyto_o.z,
                                            flyto_o.w])[2] / math.pi

        vector = np.array([flyto_p.x - current_p.x, flyto_p.y - current_p.y, flyto_p.z - current_p.z])
        norm = np.linalg.norm(vector)
        vector = vector * (1/norm) # normalize

        vector_yaw_z = 0.5 + (math.atan(-vector[0] / abs(vector[1])) / math.pi) # 0.5 + atan( x / |y|) / pi
        if(vector[1] < 0): vector_yaw_z = -vector_yaw_z

        if(norm < sub.flyto_distance_threshold): # if distance < something: move directly
            #direct move
            rospy.loginfo("direct")
            vector_nearby = vector * 0.5 # slow down
            cmd.linear = Point(vector_nearby[0],vector_nearby[1],vector_nearby[2])

            delta_yaw_z = flyto_yaw - current_yaw_z
            if(abs(delta_yaw_z) > 1): delta_yaw_z = 2 - delta_yaw_z # delta_yaw_z is the smaller angle to vector direction

            if abs(delta_yaw_z) > sub.angle_deadzone_threshold:
                #rospy.loginfo("deadzone")
                cmd.angular.z = sub.flyto_angular_limit
            if delta_yaw_z > 0:
                #rospy.loginfo("z+")
                cmd.angular.z = sub.flyto_angular_limit
            else:                   
                 #rospy.loginfo("z-")
                cmd.angular.z = -sub.flyto_angular_limit
        else:
            # turn
            rospy.loginfo("turn")

            delta_yaw_z = vector_yaw_z - current_yaw_z
            if(abs(delta_yaw_z) > 1): delta_yaw_z = 2 - delta_yaw_z # delta_yaw_z is the smaller angle to vector direction

            if(abs(delta_yaw_z) > sub.diff_angle_threshold):
                if(current_p.z < 2): cmd.linear = Point(0, 0, 1)
                else: cmd.linear = Point(0, 0, vector[2])
                
                if abs(delta_yaw_z) > sub.angle_deadzone_threshold:
                    #rospy.loginfo("deadzone")
                    cmd.angular.z = sub.flyto_angular_limit
                if delta_yaw_z > 0:
                    #rospy.loginfo("z+")
                    cmd.angular.z = sub.flyto_angular_limit
                else:
                    #rospy.loginfo("z-")
                    cmd.angular.z = -sub.flyto_angular_limit
            else:
                # move forward
                rospy.loginfo("move")
                cmd.linear = Point(sub.flyto_speed_limit,0,vector[2])
        
        
        rospy.loginfo("current: [%f,%f,%f] yaw: %f",current_p.x,current_p.y,current_p.z, current_yaw_z)
        rospy.loginfo("flyto: [%f,%f,%f] yaw: %f",flyto_p.x,flyto_p.y,flyto_p.z, flyto_yaw)
        rospy.loginfo("vec: [%f,%f,%f] vecyaw: %f, norm: %f",vector[0],vector[1],vector[2],vector_yaw_z, norm)

        sub.vel_cmd_body_frame_cb(cmd)

    def takeoff_cb(self,a):
        client.takeoffAsync()
        return [True, 'takeoff']

    def land_cb(self,a):
        client.landAsync()
        return [True, 'land']
    
    '''
    def moveTo(self, x:PoseStamped):
        client.moveToPositionAsync(x=p.x, y=p.y, z=p.z, velocity=v)
    '''

if __name__ == "__main__":
    print("python_subscriber")
    sub = AirsimSubscriber()
    
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
  

    rospy.init_node('airsim_subscriber', anonymous=True)
    
    subscriber_cmd = rospy.Subscriber('/cmd_vel', Twist, sub.vel_cmd_body_frame_cb)
    subscriber_flyTo = rospy.Subscriber('/fly_to_cmd', PoseStamped, sub.flyTo_cb)
    publisher_pose = rospy.Publisher('/pose', PoseStamped, queue_size=1)

    #service_takeoff = rospy.Service('/takeoff', SetBool, sub.takeoff_cb)
    #service_land = rospy.Service('/land', SetBool , sub.land_cb)
    #service_moveTo = rospy.Service('/droneMoveTo', PoseStamped, sub.moveTo)

    
    subscriber_pose = rospy.Subscriber('/rtabmap/localization_pose', PoseWithCovarianceStamped, sub.pose_cb)
    

    rospy.spin()
