#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class TestImagePublisher:
    def __init__(self):
        self.bridge = CvBridge()

        self.image_path = rospy.get_param("~image_path", "")
        self.topic_name = rospy.get_param("~topic", "/usb_cam/image_raw")
        self.frame_id = rospy.get_param("~frame_id", "usb_cam")
        self.publish_rate = float(rospy.get_param("~rate", 5.0))

        if not self.image_path:
            raise RuntimeError("~image_path must be set")
        if not os.path.isfile(self.image_path):
            raise RuntimeError("Image file not found: {}".format(self.image_path))

        image = cv2.imread(self.image_path, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("Failed to read image: {}".format(self.image_path))

        self.image = image
        self.publisher = rospy.Publisher(self.topic_name, Image, queue_size=1)

        rospy.loginfo(
            "Publishing test image %s to %s at %.2f Hz",
            self.image_path,
            self.topic_name,
            self.publish_rate,
        )

    def run(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            msg = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            self.publisher.publish(msg)
            rate.sleep()


def main():
    rospy.init_node("publish_test_image_node", anonymous=False)
    publisher = TestImagePublisher()
    publisher.run()


if __name__ == "__main__":
    main()
