#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from tfpose_ros.msg import Persons, Person, BodyPartElm
from tf_pose.estimator import Human, BodyPart, TfPoseEstimator
from depth_image_proc.srv import *
from geometry_msgs.msg import PointStamped


def cb_pose(data):
    # get image with pose time
    t = data.header.stamp

    if(len(data.persons) <= 0):
        return

    x = 0
    y = 0

    # ros topic to Person instance
    person = data.persons[0]
    for body_part in person.body_part:
        if(body_part.part_id == 1):
            x = body_part.x
            y = body_part.y
    if (x == 0 or y == 0):
        return
    rospy.wait_for_service('/camera/get_point_stamped')
    try:
        get_point_stamped = rospy.ServiceProxy('/camera/get_point_stamped', GetPointStamped)
        resp = get_point_stamped(float(x), float(y))
        msg_part = resp.result
        print(resp.result)
        pub_part.publish(msg_part)
        return
    except rospy.ServiceException:
        print ("Service call failed")

if __name__ == '__main__':
    rospy.loginfo('initialization+')
    rospy.init_node('TfPoseEstimatorROS-Human3dPose', anonymous=True)

    # topics params
    pose_topic = rospy.get_param('~pose', '/pose_estimator/pose')

    resize_ratio = float(rospy.get_param('~resize_ratio', '-1'))

    # publishers
    pub_part = rospy.Publisher('/part', PointStamped, queue_size = 1)
    msg_part = PointStamped()
    #msg_part.layout.dim.append(MultiArrayDimension())
    #msg_part.layout.dim.append(MultiArrayDimension())
    #msg_part.layout.dim[0].label = "height"
    #msg_part.layout.dim[1].label = "width"
    #msg_part.layout.dim[0].size = 1
    #msg_part.layout.dim[1].size = 1
    #msg_part.layout.dim[0].stride = 2
    #msg_part.layout.dim[1].stride = 1
    #msg_part.layout.data_offset = 0
    #msg_part.data = [0]*2
    # initialization

    # subscribers
    rospy.Subscriber(pose_topic, Persons, cb_pose, queue_size=1)

    # run
    rospy.spin()
