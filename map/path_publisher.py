#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import yaml
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf
from collections import deque
import copy

# ===========================
# 🧭 유틸: 거리 계산
# ===========================
def distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

def quaternion_from_yaw(yaw):
    return tf.transformations.quaternion_from_euler(0, 0, yaw)

# ===========================
# 선분과 점 거리 + 투영점 계산
# ===========================
def point_line_segment_distance(px, py, x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        return math.sqrt((px - x1)**2 + (py - y1)**2), (x1, y1)

    t = ((px - x1) * dx + (py - y1) * dy) / (dx*dx + dy*dy)
    t_clamped = max(0, min(1, t))

    closest_x = x1 + t_clamped * dx
    closest_y = y1 + t_clamped * dy
    dist = math.sqrt((px - closest_x)**2 + (py - closest_y)**2)
    return dist, (closest_x, closest_y)

# ===========================
# 📐 보간 함수: 점 사이 경로 생성
# ===========================
def interpolate_segment(x1, y1, yaw1, x2, y2, yaw2, resolution=0.05):
    dist = math.hypot(x2 - x1, y2 - y1)
    steps = max(int(dist / resolution), 1)
    path = []

    for i in range(steps + 1):
        t = i / float(steps)
        x = x1 + t * (x2 - x1)
        y = y1 + t * (y2 - y1)
        yaw = yaw1 + t * (yaw2 - yaw1)  # 간단 선형 보간

        path.append({'position': [x, y, yaw]})
    return path

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
# 📤 경로 Publish (가상 노드 포함)
# ===========================
def publish_path_from_list(node_list, topic="/my_global_path", frame_id="map"):
    pub = rospy.Publisher(topic, Path, queue_size=1, latch=True)
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    path_msg.header.stamp = rospy.Time.now()

    for node in node_list:
        pos = node['position']
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = 0.0

        q = quaternion_from_yaw(pos[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        path_msg.poses.append(pose)

    pub.publish(path_msg)

# ===========================
# 📌 노드 찾기 (가장 가까운)
# ===========================
def find_nearest_node(graph, x, y):
    min_dist = float('inf')
    nearest = None
    for node_name, node_data in graph.items():
        px, py, _ = node_data['position']
        d = distance((x, y), (px, py))
        if d < min_dist:
            min_dist = d
            nearest = node_name
    return nearest

# ===========================
# 📌 가장 가까운 엣지 + 투영점 찾기
# ===========================
def find_nearest_edge_point(graph, x, y):
    min_dist = float('inf')
    nearest_edge = None
    nearest_point = None

    for node_name, node_data in graph.items():
        x1, y1, _ = node_data['position']
        for neighbor in node_data['edges']:
            neighbor_data = graph[neighbor]
            x2, y2, _ = neighbor_data['position']

            dist, proj = point_line_segment_distance(x, y, x1, y1, x2, y2)
            if dist < min_dist:
                min_dist = dist
                nearest_edge = (node_name, neighbor)
                nearest_point = proj

    return nearest_edge, nearest_point

# ===========================
# 🎯 목표 수신 콜백
# ===========================
goal_position = None

def goal_callback(msg):
    global goal_position
    goal_position = msg.pose
    rospy.loginfo(f"목표 수신: ({goal_position.position.x:.2f}, {goal_position.position.y:.2f})")

# ===========================
# 🕐 주기적 경로 생성 (현재 위치 기반)
# ===========================
def timer_callback(event):
    global graph, tf_listener, goal_position
    if goal_position is None:
        return

    try:
        now = rospy.Time(0)
        tf_listener.waitForTransform("map", "base_link", now, rospy.Duration(1.0))
        (trans, rot) = tf_listener.lookupTransform("map", "base_link", now)
        robot_x, robot_y = trans[0], trans[1]
        robot_yaw = tf.transformations.euler_from_quaternion(rot)[2]
    except Exception as e:
        rospy.logerr(f"TF 오류: {e}")
        return

    graph_mod = copy.deepcopy(graph)

    nearest_edge, proj_point = find_nearest_edge_point(graph, robot_x, robot_y)
    if nearest_edge is None:
        rospy.logwarn("가까운 엣지를 찾지 못했습니다.")
        return

    node_a, node_b = nearest_edge

    virtual_node_name = "virtual_start"
    graph_mod[virtual_node_name] = {
        'position': [proj_point[0], proj_point[1], robot_yaw],
        'edges': [node_a, node_b]
    }
    graph_mod[node_a]['edges'].append(virtual_node_name)
    graph_mod[node_b]['edges'].append(virtual_node_name)

    goal_x = goal_position.position.x
    goal_y = goal_position.position.y
    goal_q = goal_position.orientation
    goal_yaw = tf.transformations.euler_from_quaternion([goal_q.x, goal_q.y, goal_q.z, goal_q.w])[2]
    goal_node = find_nearest_node(graph, goal_x, goal_y)

    path_nodes = bfs_path(graph_mod, virtual_node_name, goal_node)
    if path_nodes:
        full_path_nodes = []

        # 0️⃣ 현재 위치 → 투영점 보간
        interpolated = interpolate_segment(
            robot_x, robot_y, robot_yaw,
            proj_point[0], proj_point[1], robot_yaw
        )
        full_path_nodes.extend(interpolated)

        # 1️⃣ 가상 시작 노드 (투영점)
        full_path_nodes.append(graph_mod[virtual_node_name])

        # 2️⃣ BFS로 탐색된 노드
        for node in path_nodes[1:]:
            full_path_nodes.append(graph[node])

        # 3️⃣ 목표 위치
        goal_pose = {'position': [goal_x, goal_y, goal_yaw]}
        full_path_nodes.append(goal_pose)

        publish_path_from_list(full_path_nodes)
    else:
        rospy.logwarn("경로를 찾을 수 없습니다.")

# ===========================
# 🚀 메인
# ===========================
if __name__ == "__main__":
    rospy.init_node('topo_nav_goal_listener')

    yaml_path = rospy.get_param("~topology_file", "topology.yaml")
    graph = load_topology(yaml_path)
    rospy.loginfo("topology.yaml 로드 완료")

    tf_listener = tf.TransformListener()

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)
    rospy.Timer(rospy.Duration(0.1), timer_callback)

    rospy.loginfo("RViz 2D Nav Goal 기다리는 중...")

    rospy.spin()

