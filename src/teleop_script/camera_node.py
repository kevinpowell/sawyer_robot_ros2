import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ExamineImage(Node):

    def __init__(self):
        super().__init__('examine_image')

        self.mat = None
        self.sub = self.create_subscription(
            Image,
            '/D455_1/color/image_raw',
            self.image_callback,
            100)
        self.sub2 = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.image_callback2,
            100)


    def image_callback(self, msg):
        sz = (msg.height, msg.width)
        # print(msg.header.stamp)

        dirty = (self.mat is None or msg.width != self.mat.shape[1] or
                    msg.height != self.mat.shape[0] or len(self.mat.shape) < 2 or
                    self.mat.shape[2] != 3)
        if dirty:
            self.mat = np.zeros([msg.height, msg.width, 3], dtype=np.uint8)
        self.mat[:, :, 2] = np.array(msg.data[0::3]).reshape(sz)
        self.mat[:, :, 1] = np.array(msg.data[1::3]).reshape(sz)
        self.mat[:, :, 0] = np.array(msg.data[2::3]).reshape(sz)

        if self.mat is not None:
            cv2.imshow('image', self.mat)
            cv2.waitKey(5)

    def image_callback2(self, msg):
        sz = (msg.height, msg.width)
        dirty = (self.mat is None or msg.width != self.mat.shape[1] or
                    msg.height != self.mat.shape[0] or len(self.mat.shape) < 2 or
                    self.mat.shape[2] != 3)
        if dirty:
            self.mat = np.zeros([msg.height, msg.width, 3], dtype=np.uint8)
        self.mat[:, :, 2] = np.array(msg.data[0::3]).reshape(sz)
        self.mat[:, :, 1] = np.array(msg.data[1::3]).reshape(sz)
        self.mat[:, :, 0] = np.array(msg.data[2::3]).reshape(sz)

        if self.mat is not None:
            cv2.imshow('image2', self.mat)
            cv2.waitKey(5)




def main(args=None):
    rclpy.init(args=args)

    examine_image = ExamineImage()

    try:
        rclpy.spin(examine_image)
    except KeyboardInterrupt:
        pass

    examine_image.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
