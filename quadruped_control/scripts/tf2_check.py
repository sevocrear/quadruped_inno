#!/usr/bin/env python  
import rospy
import tf2_ros 
import tf
import geometry_msgs.msg
from std_msgs.msg import Float64MultiArray
import numpy as np
def get_hom_transform_btw_frames(frame_1, frame_2):

    def transform_msg_hom(msg): 
        quat = msg.transform.rotation
        tran = msg.transform.translation

        T = tf.transformations.quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        T[0:3,3] = [tran.x, tran.y, tran.z]
        return T
    try:
        trans = tf_buffer.lookup_transform(frames[0], frames[1], rospy.Time())
        T = transform_msg_hom(trans)
        return T, 1
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        # rate.sleep()
        return None, 0
if __name__ == '__main__':
    rospy.init_node('tf2_cheetah_listener')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rate = rospy.Rate(1000)
    frames = ['world', 'body']
    pub = rospy.Publisher('body_po', Float64MultiArray, queue_size=10)
    while not rospy.is_shutdown():
        T, flag = get_hom_transform_btw_frames('world', 'body')
        data_to_send = Float64MultiArray()  # the data to be sent, initialise the array
        if flag:
            data_to_send.data = T.ravel() # assign the array with the value you want to send
            pub.publish(data_to_send)
            
