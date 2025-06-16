#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

def add_cov(msg):
    # PoseWithCovarianceStamped 메시지 생성
    pose_with_cov = PoseWithCovarianceStamped()
    pose_with_cov.header = msg.header
    pose_with_cov.pose.pose = msg.pose
    
    # covariance 설정 (여기서는 단순히 0으로 초기화)
    # 실제 사용하는 경우, 의미 있는 값으로 설정해야 할 수도 있습니다.
    pose_with_cov.pose.covariance = [
        0.01, 0, 0, 0, 0, 0,    # x 축 (0.1m)
        0, 0.01, 0, 0, 0, 0,    # y 축 (0.1m)
        0, 0, 0.01, 0, 0, 0,    # z 축 (0.1m)
        0, 0, 0, 0.0001, 0, 0,  # roll 축 (0.01 rad)
        0, 0, 0, 0, 0.0001, 0,  # pitch 축 (0.01 rad)
        0, 0, 0, 0, 0, 0.0001   # yaw 축 (0.01 rad)
    ]
    return pose_with_cov

def callback(msg):
    # PoseStamped 메시지를 PoseWithCovarianceStamped로 변환
    pose = add_cov(msg)

    # 변환된 메시지를 퍼블리시
    pub.publish(pose)

def listener():
    rospy.init_node('odom_to_pose', anonymous=True)
    in_topic  = rospy.get_param("~in_topic", "car_state/odom")
    out_topic = rospy.get_param("~out_topic", "car_state/pose")
    
    rospy.Subscriber(in_topic, PoseStamped, callback)

    global pub
    pub = rospy.Publisher(out_topic, PoseWithCovarianceStamped, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    listener()
