import numpy as np
import cv2
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose

bridge = CvBridge()
hoop_pose = Pose()

def receive_image(image_data):
    global hoop_pose

    cv2_image = bridge.imgmsg_to_cv2(image_data, desired_encoding="passthrough")

    cv2.imshow("Image", cv2_image)
    cv2.waitKey(2)
    # cv2.waitKey(0) & 0xFF

    hsv = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2HSV)

    # green_lower = np.array([90, 0, 0])
    # green_upper = np.array([110, 150, 150])
    green_lower = np.array([90, 20, 80])
    green_upper = np.array([110, 150, 120])
    # green_lower = np.array([80, 0, 0])
    # green_upper = np.array([120, 255, 255])

    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    # green_mask = cv2.dilate(green_mask, None, iterations=10)
    # green_mask = cv2.erode(green_mask, None, iterations=1)
    # green_mask = cv2.dilate(green_mask, None, iterations=5)

    # gray_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2GRAY)
    # hough_circles = cv2.HoughCircles(gray_image, cv2.HOUGH_GRADIENT, 
    #                                  1, 640, param1=50, param2=30, 
    #                                  minRadius=30, maxRadius=320)

    hough_circles = cv2.HoughCircles(green_mask, cv2.HOUGH_GRADIENT, 
                                     1, 640, param1=50, param2=30, 
                                     minRadius=30, maxRadius=320)

    green_mask = green_mask / 4
    # hough_circles = np.uint8(np.around(hough_circles))
    if hough_circles is not None:
        hough_flag = True
        hough_circle = hough_circles[0]
    else:
        hough_flag = False

    green_contours = cv2.findContours(green_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(green_contours) > 0 and hough_flag:
        # green_M = cv2.moments(max(green_contours, key=cv2.contourArea))
        # green_center = Point(int(green_M['m10'] / green_M['m00']), 
        #                      int(green_M['m01'] / green_M['m00']), 0)
        # print("Hoop detected!")
        largest_contour = max(green_contours, key=cv2.contourArea)
        # print(largest_contour.shape)
        min_width = min(largest_contour[:, 0, 0])
        max_width = max(largest_contour[:, 0, 0])
        min_height = min(largest_contour[:, 0, 1])
        max_height = max(largest_contour[:, 0, 1])
        width = max_width - min_width
        height = max_height - min_height
        distance_to_hoop = 3547.53 * height ** -1.0524578 

        center_x = (min_width + max_width) / 2
        center_y = (min_height + max_height) / 2

        # c920 viewing angle: 78 degrees
        angle_to_hoop = 78 * (center_x - 320.) / 640

        # Perpendicular width/height ratio: 0.888, linear approximation
        hoop_orientation = 90 * ((1. * width / height) / 0.888)

        if (abs(center_x - hough_circle[0][0]) < hough_circle[0][2] / 2 
            and abs(center_y - hough_circle[0][1]) < hough_circle[0][2] / 2):
            for point in largest_contour:
                green_mask[point[0, 1], point[0, 0]] = 255
            cv2.circle(green_mask, (hough_circle[0][0], hough_circle[0][1]), 
                       hough_circle[0][2], (255, 255, 255), 0)
            print(distance_to_hoop, angle_to_hoop, hoop_orientation)

            rel_x = distance_to_hoop * np.cos(np.pi * angle_to_hoop / 180.)
            rel_y = distance_to_hoop * np.sin(np.pi * angle_to_hoop / 180.)
            green_center = Point(rel_x, rel_y, 0)
            print(green_center)
        else:
            green_center = Point(0, 0, 0)
    else:
        green_center = Point(0, 0, 0)

    green_center = Point()
    cv2.imshow("Mask", green_mask)
    cv2.waitKey(2)

    hoop_pose = Pose()
    hoop_pose.position = green_center
    return

def pub_sub_init():
    global hoop_pose

    pub_hoop_pose = rospy.Publisher('/hoop/pose', Pose, queue_size=1)
    sub_image = rospy.Subscriber('/camera/rgb/image_raw/image', Image, receive_image, queue_size=1)

    rospy.init_node('hoop_detector', anonymous=True)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub_hoop_pose.publish(hoop_pose)
        rate.sleep()

    return

if __name__ == "__main__":
    try:
        pub_sub_init()
    except rospy.ROSInterruptException:
        pass

    cv2.destroyAllWindows()
