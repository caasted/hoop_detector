import glob
import numpy as np
import cv2
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

image_paths = list(glob.glob("images/*.png"))
image_paths.sort()

max_dist = int((2016 - 485) / 2) # 27 ft
min_dist = int((3472 - 486) / 2) # 9 ft
# min_dist = int((2516 - 486) / 2)
image_paths = image_paths[max_dist:min_dist]

bridge = CvBridge()

def pub_init():
    pub_image = rospy.Publisher('/camera/rgb/image_raw/image', Image, queue_size=1)

    rospy.init_node('test_image_publisher', anonymous=True)

    index = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        cv_image = cv2.imread(image_paths[index % len(image_paths)])

        # Mask off cabinet
        cv_image[160:270, 0:50, :] = 0
        # cv_image[160:270, 0:150, :] = 0

        # cv2.imshow("Image", cv_image)
        # cv2.waitKey(2) # & 0xFF
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")

        # msg = CompressedImage()
        # msg.header.stamp = rospy.Time.now()
        # msg.data = image.tostring()

        pub_image.publish(image_message)
        print(index)
        index += 1
        rate.sleep()

    # cv2.destroyAllWindows()
    return

if __name__ == "__main__":
    try:
        pub_init()
    except rospy.ROSInterruptException:
        pass

