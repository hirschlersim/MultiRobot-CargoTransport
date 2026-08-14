#! /usr/bin/env python3
"""Detects yellow objects in the camera stream and publishes their normalized
image-plane position on x_Obj_Yellow / y_Obj_Yellow."""
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
    """Draw the region-of-interest outline on the image."""
    pts = np.array([[685, 50], [685, 100], [625, 100], [625, 50]], np.int32)
    pts = pts.reshape((-1, 1, 2))
    cv2.polylines(image, [pts], True, (0, 165, 255), 3)  # closed polygon


def process_image(image):
    img_copy = np.copy(image)
    hsv_frame = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
    region_of_interest(img_copy)

    lower_bound = np.array([20, 55, 195])
    upper_bound = np.array([30, 255, 255])
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

                img_copy = cv2.rectangle(img_copy, (x1, y1), (x1 + x2, y1 + y2), (0, 255, 255), 2)
                cv2.putText(img_copy, "Y", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                img_copy = cv2.circle(img_copy, center_point, radius=2, color=(0, 0, 255), thickness=3)

        for point in object_centers:
            scaled_x = (point[0] - centroid[0]) / centroid[0]
            scaled_y = (centroid[1] - point[1]) / centroid[1]

            # empirical calibration offset for this camera/tag setup
            scaled_x = (scaled_x + 0.135) * 1.343
            scaled_y = (scaled_y + 0.025) * -1.138

            x_pub.publish(scaled_x)
            y_pub.publish(scaled_y)
            print(f"x:{scaled_x:+3.4f} \t y:{scaled_y:+3.4f}")

    cv2.imshow("Stream: ", img_copy)
    cv2.waitKey(1)


if __name__ == '__main__':
    rospy.init_node("image_subscriber_node", anonymous=True)

    rospy.Subscriber("/camera_rect/image_rect", Image, image_callback)

    pub = rospy.Publisher('imagetimer', Image, queue_size=10)
    x_pub = rospy.Publisher('x_Obj_Yellow', Float32, queue_size=10)
    y_pub = rospy.Publisher('y_Obj_Yellow', Float32, queue_size=10)

    loop_rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if image is not None:
            process_image(image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("break")
                break
            pub.publish(br.cv2_to_imgmsg(image))
        loop_rate.sleep()
