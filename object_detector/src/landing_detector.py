#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class LandingDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/quad_f450_camera/camera_link/raw_image", Image, self.image_callback)
        self.landing_pub = rospy.Publisher("/landing_location", PointStamped, queue_size=1)

        # Define pixel density of the camera (in pixels/meter)
        self.pixel_density = 207.06

    def image_callback(self, data):
        try:
            # Convert image to OpenCV format for processing
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Get image dimensions (width, height, number of channels)
            height, width, channels = cv_image.shape

            # Define color range for target object
            lower_white = np.array([0,0,200])
            upper_white = np.array([255,30,255])
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            white_mask = cv2.inRange(hsv_image, lower_white, upper_white)

            # Find contours in mask
            contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Convert pixel coordinates to metric coordinates
                m_x = cx / self.pixel_density
                m_y = (height - cy) / self.pixel_density

                # Draw contours and center point on the image
                cv2.drawContours(cv_image, [c], 0, (0,255,0), 3)
                cv2.circle(cv_image, (cx,cy), 5, (0,0,255), -1)

                # Show image on screen
                cv2.imshow("Image", cv_image)
                cv2.waitKey(1)

                # Create and publish ROS message containing landing location
                landing_location = PointStamped()
                landing_location.header.stamp = data.header.stamp
                landing_location.header.frame_id = "base_link"
                landing_location.point.x = m_x
                landing_location.point.y = m_y
                landing_location.point.z = 0
                self.landing_pub.publish(landing_location)

                # Print coordinates to terminal
                rospy.loginfo("Landing location: x=%f m, y=%f m, z=%f m", landing_location.point.x, landing_location.point.y, landing_location.point.z)

        except cv2.error as e:
            print(e)

def main():
    rospy.init_node('landing_detector', anonymous=True)
    landing_detector = LandingDetector()
    rospy.spin()

if __name__ == '__main__':
    main()

