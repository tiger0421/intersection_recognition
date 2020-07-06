#!/usr/bin/env python
import rospy
import rosparam
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

import tensorflow as tf
from tensorflow import keras

import time


class Variables:
    def __init__(self):
        rospy.loginfo("start init")

        self.rate = rospy.Rate(1)
        self.TRAIN_DATA_FILES = ['dead_end', 'left', 'right', 'straight', 'threeway_left', 'threeway_center', 'threeway_right']
        NUM_CLASSES = len(self.TRAIN_DATA_FILES)

        time.sleep(3)
        #rosparam.set_param("model_full_path", "/home/sima/catkin_tensorflow_ws/src/tensorflow/model/NN.h5")
        model_full_path = rosparam.get_param("tensorflow/model_full_path")
        rospy.loginfo("Out txt")
        rospy.loginfo(model_full_path)
        self.model = keras.models.load_model(model_full_path)

        rospy.loginfo("finish init")

def callback(data):
    start = time.time()
    h_idx = Float32MultiArray()
    input_pointclouds = np.array(data.ranges, dtype=np.float32)

    h_idx.data = variables.model.predict(input_pointclouds.reshape(1, len(input_pointclouds)))

    publisher = rospy.Publisher('hypothesis', Float32MultiArray, queue_size=10)
    publisher.publish(h_idx)

    end = time.time()
    print("time: ", end-start)

    rospy.loginfo(variables.TRAIN_DATA_FILES[np.argmax(h_idx.data)])
    variables.rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('tensorflow', anonymous=True)
        variables = Variables()
        rospy.Subscriber("/scan2", LaserScan, callback)
        hz = rosparam.get_param("adjust_hz/scan_hz")
        loop_rate = rospy.Rate(hz)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("except occur")
        pass
