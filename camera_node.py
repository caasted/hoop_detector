import cv2
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()

def pub_sub_init():
    pub_image = rospy.Publisher('/camera/rgb/image_raw/image', Image, queue_size=1)

    rospy.init_node('camera_node', anonymous=True)

    cap = cv2.VideoCapture(0)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ret, cv_image = cap.read()
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
        pub_image.publish(image_message)

    return

if __name__ == "__main__":
    try:
        pub_sub_init()
    except rospy.ROSInterruptException:
        pass

