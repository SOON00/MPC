#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import heapq
import tf2_ros
import tf2_geometry_msgs

class DijkstraPlanner:
    def __init__(self):
        rospy.init_node('dijkstra_planner')

        self.costmap = None
        self.path_pub = rospy.Publisher('/dijkstra_global_path', Path, queue_size=1)
        self.costmap_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.costmap_cb)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.start = None
        self.goal = None

        self.obstacle_threshold = 50
        rospy.loginfo("Dijkstra planner node started.")

        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)

    def costmap_cb(self, msg):
        self.costmap = msg

    def goal_cb(self, msg):
        self.goal = msg

    def timer_callback(self, event):
        self.try_plan()

    def get_robot_pose(self):
        if self.costmap is None:
            return None
        try:
            now = rospy.Time.now()
            frame = self.costmap.header.frame_id
            timeout = rospy.Duration(0.01)
            if not self.tf_buffer.can_transform(frame, "base_link", now, timeout):
                rospy.logwarn("Transform from base_link to costmap frame not ready")
                return None
            trans = self.tf_buffer.lookup_transform(frame, "base_link", now, timeout)
            pose = PoseStamped()
            pose.header.frame_id = trans.header.frame_id
            pose.header.stamp = trans.header.stamp
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            return pose
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF lookup failed: {e}")
            return None

    def try_plan(self):
        if self.costmap is None or self.goal is None:
            return

        self.start = self.get_robot_pose()
        if self.start is None:
            return

        try:
            costmap_frame = self.costmap.header.frame_id
            timeout = rospy.Duration(0.01)

            if not self.tf_buffer.can_transform(costmap_frame, self.start.header.frame_id, rospy.Time(0), timeout):
                rospy.logwarn("Transform from start frame not ready")
                return
            start_tf = self.tf_buffer.transform(self.start, costmap_frame, timeout)

            if not self.tf_buffer.can_transform(costmap_frame, self.goal.header.frame_id, rospy.Time(0), timeout):
                rospy.logwarn("Transform from goal frame not ready")
                return
            goal_tf = self.tf_buffer.transform(self.goal, costmap_frame, timeout)

            ox, oy = self.costmap.info.origin.position.x, self.costmap.info.origin.position.y
            res = self.costmap.info.resolution
            w, h = self.costmap.info.width, self.costmap.info.height

            costmap_grid = np.array(self.costmap.data).reshape((h, w))

            start_idx = self.world_to_grid(start_tf.pose.position.x, start_tf.pose.position.y, ox, oy, res)
            goal_idx = self.world_to_grid(goal_tf.pose.position.x, goal_tf.pose.position.y, ox, oy, res)

            path_indices = self.dijkstra(start_idx, goal_idx, costmap_grid, self.obstacle_threshold)

            if not path_indices:
                rospy.logwarn("No path found by Dijkstra!")
                return

            path_msg = Path()
            path_msg.header.frame_id = costmap_frame
            path_msg.header.stamp = rospy.Time.now()

            for gx, gy in path_indices:
                px, py = self.grid_to_world(gx, gy, ox, oy, res)
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = px
                pose.pose.position.y = py
                pose.pose.position.z = 0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            rospy.loginfo("Published global path with %d points", len(path_indices))

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF error: {e}")

    def world_to_grid(self, x, y, ox, oy, res):
        gx = int((x - ox) / res)
        gy = int((y - oy) / res)
        return gx, gy

    def grid_to_world(self, gx, gy, ox, oy, res):
        x = ox + (gx + 0.5) * res
        y = oy + (gy + 0.5) * res
        return x, y

    def dijkstra(self, start, goal, grid, obstacle_thresh):
        w, h = grid.shape[1], grid.shape[0]
        visited = set()
        dist = {}
        prev = {}

        dist[start] = 0
        queue = []
        heapq.heappush(queue, (0, start))

        while queue:
            current_dist, current = heapq.heappop(queue)
            if current in visited:
                continue
            visited.add(current)

            if current == goal:
                path = []
                while current != start:
                    path.append(current)
                    current = prev[current]
                path.append(start)
                path.reverse()
                return path

            x, y = current
            neighbors = self.get_neighbors(x, y, w, h)
            for nx, ny in neighbors:
                if grid[ny, nx] >= obstacle_thresh:
                    continue
                if (nx, ny) in visited:
                    continue

                cost = current_dist + np.hypot(nx - x, ny - y)
                if (nx, ny) not in dist or cost < dist[(nx, ny)]:
                    dist[(nx, ny)] = cost
                    prev[(nx, ny)] = current
                    heapq.heappush(queue, (cost, (nx, ny)))

        return []

    def get_neighbors(self, x, y, w, h):
        nbrs = []
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1), (-1,-1),(1,-1),(-1,1),(1,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h:
                nbrs.append((nx, ny))
        return nbrs

if __name__ == "__main__":
    planner = DijkstraPlanner()
    rospy.spin()

