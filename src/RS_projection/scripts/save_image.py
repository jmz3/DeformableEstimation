import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    try:
        # Convert the ROS Image message to a OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        #cv2.imshow("figure1", cv_image)
        #cv2.waitKey(0)
        # Save the image as PNG
        cv2.imwrite("received_image.png", cv_image)
        rospy.loginfo("Image saved as received_image.png")
    except Exception as e:
        rospy.logerr(e)

if __name__ == "__main__":
    rospy.init_node('image_listener')

    # Replace '/camera/image_topic' with the actual topic you want to subscribe to
    image_topic = '/camera/color/image_raw'

    rospy.Subscriber(image_topic, Image, image_callback)

    rospy.spin()