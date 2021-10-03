#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import message_filters

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from semseg.semseg import SemanticSegmentator


class VisualizerNode:

    def __init__(self):
        rospy.init_node('visualizer_node')

        image_sub = message_filters.Subscriber('image', Image)
        segmentation_sub = message_filters.Subscriber('segmentation', Image)

        self.ts = message_filters.TimeSynchronizer([image_sub, segmentation_sub], 10)
        self.ts.registerCallback(self.on_image_segmentation)

        self.pub_segmentation_color = rospy.Publisher('segmentation_color', Image, queue_size=10)

        self.br = CvBridge()


    def on_image_segmentation(self, image_msg : Image, segm_msg : Image):
        image = self.br.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        segmentation = self.br.imgmsg_to_cv2(segm_msg, desired_encoding='mono8')

        segmentation_color = SemanticSegmentator.colorize(segmentation)

        segm_color_msg = self.br.cv2_to_imgmsg(segmentation_color, 'rgb8')
        segm_color_msg.header = segm_msg.header

        self.pub_segmentation_color.publish(segm_color_msg)


def main(args=None):
    node = VisualizerNode()
    rospy.spin()


if __name__ == '__main__':
    main()
