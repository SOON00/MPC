#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import yaml
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class WaypointFollower:
    def __init__(self):
        self.waypoints = []
        self.current_idx = 0

        self.tf_listener = tf.TransformListener()
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
        self.path_sub = rospy.Subscriber('/my_global_path', Path, self.path_callback)

    def path_callback(self, msg):
        self.waypoints = []
        for pose_stamped in msg.poses:
            pos = pose_stamped.pose.position
            ori = pose_stamped.pose.orientation
            yaw = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])[2]
            self.waypoints.append({'position': [pos.x, pos.y, yaw]})
        self.current_idx = 0
        rospy.loginfo(f"🛤️ 경로 수신, waypoint 수: {len(self.waypoints)}")
        self.publish_current_goal()

    def publish_current_goal(self):
        if self.current_idx >= len(self.waypoints):
            rospy.loginfo("✅ 모든 waypoint 도달 완료")
            return

        # Publisher가 연결될 때까지 대기
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while self.goal_pub.get_num_connections() == 0:
            if rospy.Time.now() > timeout:
                rospy.logwarn("⚠️ /move_base_simple/goal에 구독자 연결 실패")
                return
            rospy.sleep(0.1)

        wp = self.waypoints[self.current_idx]['position']
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose.position.x = wp[0]
        goal_msg.pose.position.y = wp[1]
        goal_msg.pose.position.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, wp[2])
        goal_msg.pose.orientation.x = q[0]
        goal_msg.pose.orientation.y = q[1]
        goal_msg.pose.orientation.z = q[2]
        goal_msg.pose.orientation.w = q[3]

        self.goal_pub.publish(goal_msg)
        rospy.loginfo(f"📍 waypoint {self.current_idx + 1}/{len(self.waypoints)} publish: ({wp[0]:.2f}, {wp[1]:.2f})")

    def run(self):
        rate = rospy.Rate(2)  # 2Hz
        while not rospy.is_shutdown():
            try:
                # 현재 로봇 위치와 waypoint 거리 체크해서 다음 waypoint로 전진
                now = rospy.Time(0)
                self.tf_listener.waitForTransform("map", "base_link", now, rospy.Duration(1.0))
                (trans, rot) = self.tf_listener.lookupTransform("map", "base_link", now)
                robot_x, robot_y = trans[0], trans[1]

                if self.current_idx < len(self.waypoints):
                    wp = self.waypoints[self.current_idx]['position']
                    dist = ((robot_x - wp[0])**2 + (robot_y - wp[1])**2)**0.5

                    if dist < 0.3:  # waypoint 도달 임계값 (0.3m)
                        rospy.loginfo(f"✅ waypoint {self.current_idx + 1} 도달")
                        self.current_idx += 1
                        self.publish_current_goal()

                rate.sleep()
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
                rospy.logwarn(f"TF 예외: {e}")

if __name__ == "__main__":
    rospy.init_node('waypoint_follower_node')
    follower = WaypointFollower()
    rospy.loginfo("🚀 WaypointFollower 시작")
    follower.run()

