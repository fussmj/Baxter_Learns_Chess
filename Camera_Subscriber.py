#!/usr/bin/python2
import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
bridge = cv_bridge.CvBridge() # Initialize CV Bridge object
# Callback function to subscribe to images
def image_callback(ros_img):
    # Convert received image message to OpenCv image
    cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
    cv2.imshow('Image', cv_image) # display image
    cv2.waitKey(1)
if __name__ == '__main__':
    rospy.init_node('Camera_Subscriber',anonymous=True) # Initialze ROS node
    # Subscribe to right_hand_camera image topic
    rospy.Subscriber('/cameras/right_hand_camera/image', Image, image_callback)
    rospy.spin() # sleep
    cv2.destroyAllWindows() # Destroy CV image window on shut_down
