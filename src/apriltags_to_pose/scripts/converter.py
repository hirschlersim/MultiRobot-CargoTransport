#!/usr/bin/env python2
"""Republishes AprilTag detections as a plain PoseArray for RViz/marker consumers."""
import rospy
import std_msgs.msg
from geometry_msgs.msg import PoseArray
from apriltag_ros.msg import AprilTagDetectionArray

# Static offset between the tag frame and the map, applied via:
# rosrun tf static_transform_publisher 1.7 1.5 -23.68 0.0 0.0 0.0 1.0 map my_frame 10
FRAME_ID = 'my_frame'


def detections_to_pose_array(data, publisher):
    """Convert an AprilTagDetectionArray into a PoseArray and publish it."""
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = FRAME_ID

    pose_array = PoseArray()
    pose_array.header = header
    for detection in data.detections:
        pose_array.poses.append(detection.pose.pose.pose)
    publisher.publish(pose_array)


def listener():
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('tags_in_markers', PoseArray, queue_size=10)
    rospy.Subscriber("tag_detections", AprilTagDetectionArray, lambda data: detections_to_pose_array(data, pub))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
