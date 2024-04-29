import rospy
from sensor_msgs.msg import Image,CameraInfo
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped , PoseStamped, Point

from cv_bridge import CvBridge
import airsim
import cv2
import numpy as np
from iottalk_utils import iottalk_helper
import base64

CLAHE_ENABLED = False  # when enabled, RGB image is enhanced using CLAHE

CAMERA_FX = 320
CAMERA_FY = 320
CAMERA_CX = 320
CAMERA_CY = 240

CAMERA_K1 = -0.000591
CAMERA_K2 = 0.000519
CAMERA_P1 = 0.000001
CAMERA_P2 = -0.000030
CAMERA_P3 = 0.0

IMAGE_WIDTH = 640  # resolution should match values in settings.json
IMAGE_HEIGHT = 480



class KinectPublisher:

    useIottalk: bool = True

    def __init__(self):
        self.bridge_rgb = CvBridge()
        self.msg_rgb = Image()
        self.bridge_d = CvBridge()
        self.msg_d = Image()
        self.msg_info = CameraInfo()
        self.msg_tf = TFMessage()

    def getDepthImage(self,response_d):
        img_depth = np.array(response_d.image_data_float, dtype=np.float32)
        img_depth = img_depth.reshape(response_d.height, response_d.width)
        return img_depth

    def getRGBImage(self,response_rgb):
        img1d = np.frombuffer(response_rgb.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(response_rgb.height, response_rgb.width, 3)
        img_rgb = img_rgb[..., :3][..., ::-1]
        return img_rgb

    def enhanceRGB(self,img_rgb):
        lab = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2LAB)
        lab_planes = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(10, 10))
        lab_planes[0] = clahe.apply(lab_planes[0])
        lab = cv2.merge(lab_planes)
        img_rgb = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        return img_rgb

    def GetCurrentTime(self):
        self.ros_time = rospy.Time.now()

    def CreateRGBMessage(self,img_rgb):
        self.msg_rgb.header.stamp = self.ros_time
        self.msg_rgb.header.frame_id = "camera_rgb_optical_frame"
        self.msg_rgb.encoding = "bgr8"
        self.msg_rgb.height = IMAGE_HEIGHT
        self.msg_rgb.width = IMAGE_WIDTH
        self.msg_rgb.data = self.bridge_rgb.cv2_to_imgmsg(img_rgb, "bgr8").data
        self.msg_rgb.is_bigendian = 0
        self.msg_rgb.step = self.msg_rgb.width * 3
        return self.msg_rgb

    def CreateDMessage(self,img_depth):
        self.msg_d.header.stamp = self.ros_time
        self.msg_d.header.frame_id = "camera_depth_optical_frame"
        self.msg_d.encoding = "32FC1"
        self.msg_d.height = IMAGE_HEIGHT
        self.msg_d.width = IMAGE_WIDTH
        self.msg_d.data = self.bridge_d.cv2_to_imgmsg(img_depth, "32FC1").data
        self.msg_d.is_bigendian = 0
        self.msg_d.step = self.msg_d.width * 4
        return self.msg_d
    

if __name__ == "__main__":
    print("python_publisher")
    pub = KinectPublisher()

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    rospy.init_node('airsim_publisher', anonymous=True)
    
    rate = rospy.Rate(30)  # 30hz
    iottalk = iottalk_helper('publisher', ['twin_img'], '39393900')
    print('register')
    response = iottalk.register()
    print(response)

    publisher_test_rate = rospy.Publisher('/test_rate', Int32, queue_size=1)


    while not rospy.is_shutdown():
    #while False:
        responses = client.simGetImages([airsim.ImageRequest("front_center_custom", airsim.ImageType.DepthPlanar, True, False),
                                         airsim.ImageRequest("front_center_custom", airsim.ImageType.Scene, False, False)])
        img_depth = pub.getDepthImage(responses[0])
        img_rgb = pub.getRGBImage(responses[1])

        if CLAHE_ENABLED:
            img_rgb = pub.enhanceRGB(img_rgb)
        
        pub.GetCurrentTime()
        msg_rgb = pub.CreateRGBMessage(img_rgb)
        msg_d = pub.CreateDMessage(img_depth)

        b64_msg_d_bytes = base64.b64encode(msg_d.data)
        b85_msg_d_bytes = base64.b85encode(msg_d.data)
        #result = iottalk.push(data_str = b64_msg_d_bytes.decode('ascii'), feature="img_d")
        # b64bytes -> b64string
        b64_msg_rgb_bytes = base64.b64encode(msg_rgb.data) 
        b85_msg_rgb_bytes = base64.b85encode(msg_rgb.data)
        #result = iottalk.push_twin(b64_msg_d_bytes.decode('ascii'), b64_msg_rgb_bytes.decode('ascii'), feature='twin_img')
        result = iottalk.push_twin(b64_msg_d_bytes.decode('ascii'), b64_msg_rgb_bytes.decode('ascii'), feature='twin_img')
        print(result)
        #print('size raw: (d / rgb)' + str(len(msg_d.data)) + ' ' + str(len(msg_rgb.data)))
        #print('size b64: (d / rgb)' + str(len(b64_msg_d_bytes)) + ' ' + str(len(b64_msg_rgb_bytes)))
        #print('size b64: (d / rgb)' + str(len(b85_msg_d_bytes)) + ' ' + str(len(b85_msg_rgb_bytes)))

        publisher_test_rate.publish(39)

        del pub.msg_info.D[:]
        del pub.msg_tf.transforms[:]

        rate.sleep()