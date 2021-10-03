#!/usr/bin/env python3

import rospy
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from semseg.semseg import SemanticSegmentator
from inference_speed_meter import InferenceSpeedMeter


class SemSegNode:

    def __init__(self) -> None:
        rospy.init_node('semseg_node')

        self.weights = rospy.get_param('~weights')

        self.treshold = rospy.get_param('~treshold', 0.5)

        self.segmentator = SemanticSegmentator(self.weights)

        self.br = CvBridge()

        self.sub_image = rospy.Subscriber('image', Image, self.on_image, queue_size=10)
        self.pub_segmentation = rospy.Publisher('segmentation', Image, queue_size=10)

        self.speed_meter = InferenceSpeedMeter()


    def on_image(self, image_msg : Image):
        image = self.br.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        self.speed_meter.start()
        segmentation = self.segmentator.inference(image, self.treshold)
        self.speed_meter.stop()

        segmentation_msg = self.br.cv2_to_imgmsg(segmentation, 'mono8')
        segmentation_msg.header = image_msg.header

        self.pub_segmentation.publish(segmentation_msg)


def main(args=None):
    node = SemSegNode()
    rospy.spin()


if __name__ == '__main__':
    main()
