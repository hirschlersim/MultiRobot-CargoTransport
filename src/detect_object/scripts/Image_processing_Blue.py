#! /usr/bin/env python3
"""Detects blue objects in the camera stream and publishes their normalized
image-plane position on x_Obj_Blue / y_Obj_Blue."""
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32

# Global for the latest image received from the camera topic
image = None
br = CvBridge()


def image_callback(msg):
    global image
    image = br.imgmsg_to_cv2(msg)


def region_of_interest(image):
    """Draw the region-of-interest outline and reference point on the image."""
    pts = np.array([[550, 310], [550, 370], [660, 370], [660, 310]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    cv2.polylines(image, [pts], True, (0, 165, 255), 3)  # closed polygon
    cv2.circle(image, (458, 218), 5, (0, 160, 255), -1)


def process_image(image):
    img_copy = np.copy(image)
    hsv_frame = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
    region_of_interest(img_copy)

    lower_bound = np.array([44, 86, 100])
    upper_bound = np.array([115, 213, 255])
    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

    kernel = np.ones((5, 5), "uint8")
    mask_bb = cv2.dilate(mask, kernel)

    centroid = np.array([458.0, 218.0])

    contours, _ = cv2.findContours(mask_bb, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    object_centers = []

    if len(contours) != 0:
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= 75:
                x1, y1, x2, y2 = cv2.boundingRect(contour)  # coordinates of our object
                center_point = (int(x1 + x2 / 2), int(y1 + y2 / 2))
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    center_x = float(M["m10"] / M["m00"])
                    center_y = float(M["m01"] / M["m00"])
                    object_centers.append((center_x, center_y))

                img_copy = cv2.rectangle(img_copy, (x1, y1), (x1 + x2, y1 + y2), (255, 0, 0), 2)
                cv2.putText(img_copy, "B", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                img_copy = cv2.circle(img_copy, center_point, radius=2, color=(0, 0, 255), thickness=3)

        for point in object_centers:
            scaled_x = (point[0] - centroid[0]) / centroid[0]
            scaled_y = (centroid[1] - point[1]) / centroid[1]

            x_pub.publish(scaled_x)
            y_pub.publish(scaled_y)
            print(f"x:{scaled_x:0.4} \t\t y:{scaled_y:0.4}")

    cv2.imshow("Stream: ", img_copy)
    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node("image_subscriber_node", anonymous=True)

    rospy.Subscriber("/camera_rect/image_rect", Image, image_callback)

    pub = rospy.Publisher('imagetimer', Image, queue_size=10)
    x_pub = rospy.Publisher('x_Obj_Blue', Float32, queue_size=10)
    y_pub = rospy.Publisher('y_Obj_Blue', Float32, queue_size=10)

    loop_rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        if image is not None:
            process_image(image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("break")
                break
            pub.publish(br.cv2_to_imgmsg(image))
        loop_rate.sleep()
