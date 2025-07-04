#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import yaml
import math
import tf
import actionlib
from collections import deque
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped

# ===========================
# 🧭 유틸: 거리 계산
# ===========================
def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def quaternion_from_yaw(yaw):
    return tf.transformations.quaternion_from_euler(0, 0, yaw)

# ===========================
# 🧠 그래프 로딩
# ===========================
def load_topology(yaml_path):
    with open(yaml_path, 'r') as f:
        return yaml.safe_load(f)

# ===========================
# 🔍 BFS 탐색
# ===========================
def bfs_path(graph, start, goal):
    queue = deque([[start]])
    visited = set()

    while queue:
        path = queue.popleft()
        node = path[-1]
        if node == goal:
            return path
        if node not in visited:
            visited.add(node)
            for neighbor in graph[node]['edges']:
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)
    return None

# ===========================
# 🎯 WaypointFollower 클래스
# ===========================
class WaypointFollower:
    def __init__(self, graph):
        self.graph = graph
        self.tf_listener = tf.TransformListener()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        self.waypoints = []
        self.current_index = 0
        self.goal_active = False

        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

    def goal_callback(self, msg):
        try:
            now = rospy.Time(0)
            self.tf_listener.waitForTransform("map", "base_link", now, rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform("map", "base_link", now)
            robot_x, robot_y = trans[0], trans[1]
            robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
        except Exception as e:
            rospy.logerr(f"TF 오류: {e}")
            return

        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        goal_q = msg.pose.orientation
        goal_yaw = tf.transformations.euler_from_quaternion([goal_q.x, goal_q.y, goal_q.z, goal_q.w])[2]

        start_node = self.find_nearest_node(robot_x, robot_y)
        goal_node = self.find_nearest_node(goal_x, goal_y)

        rospy.loginfo(f"🏁 시작 노드: {start_node}, 목표 노드: {goal_node}")

        path_nodes = bfs_path(self.graph, start_node, goal_node)
        if not path_nodes:
            rospy.logwarn("❌ 경로를 찾을 수 없습니다.")
            return

        rospy.loginfo(f"🛣️ 경로 발견: {path_nodes}")

        # 웨이포인트 리스트 초기화
        self.waypoints = []

        # 현재 위치 Pose 추가 (실제 로봇 현재 위치)
        self.waypoints.append({'position': [robot_x, robot_y, robot_yaw]})

        # 경로 노드들 위치 추가
        for node in path_nodes:
            self.waypoints.append(self.graph[node])

        # 목표 위치 Pose 추가 (사용자가 지정한 최종 목표)
        self.waypoints.append({'position': [goal_x, goal_y, goal_yaw]})

        self.current_index = 0
        self.send_next_goal()

    def find_nearest_node(self, x, y):
        min_dist = float('inf')
        nearest = None
        for node_name, node_data in self.graph.items():
            px, py, _ = node_data['position']
            d = distance((x, y), (px, py))
            if d < min_dist:
                min_dist = d
                nearest = node_name
        return nearest

    def send_next_goal(self):
        if self.current_index >= len(self.waypoints):
            rospy.loginfo("✅ 모든 웨이포인트 도달 완료")
            return

        pos = self.waypoints[self.current_index]['position']
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = "map"
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        goal_msg.target_pose.pose.position.x = pos[0]
        goal_msg.target_pose.pose.position.y = pos[1]
        q = quaternion_from_yaw(pos[2])
        goal_msg.target_pose.pose.orientation.x = q[0]
        goal_msg.target_pose.pose.orientation.y = q[1]
        goal_msg.target_pose.pose.orientation.z = q[2]
        goal_msg.target_pose.pose.orientation.w = q[3]

        rospy.loginfo(f"➡️ 웨이포인트 {self.current_index+1}/{len(self.waypoints)} 목표 전송: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
        self.client.send_goal(goal_msg, done_cb=self.done_cb)
        self.goal_active = True

    def done_cb(self, status, result):
        if status == 3:  # SUCCEEDED
            rospy.loginfo(f"✅ 웨이포인트 {self.current_index+1} 도달 성공")
            self.current_index += 1
            self.send_next_goal()
        else:
            rospy.logwarn(f"❌ 웨이포인트 {self.current_index+1} 도달 실패 또는 취소됨 (status={status})")
            # 실패시 재시도 or 다른 처리 로직을 넣어도 됨
            self.goal_active = False

# ===========================
# 🚀 메인
# ===========================
if __name__ == "__main__":
    rospy.init_node('topo_waypoint_follower')

    yaml_path = rospy.get_param("~topology_file", "topology.yaml")
    graph = load_topology(yaml_path)
    rospy.loginfo("✅ topology.yaml 로드 완료")

    wf = WaypointFollower(graph)

    rospy.spin()

