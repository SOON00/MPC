import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from PIL import Image

# Map meta info
resolution = 0.05
origin = [-29.993122, -10.065166, 0.0]

# Load image height (pixels)
pgm_path = "/home/soon/map/warehouse.pgm"
img = Image.open(pgm_path)
image_height = img.size[1]

def load_nodes_from_txt(file_path):
    nodes = []
    with open(file_path, 'r') as f:
        for idx, line in enumerate(f):
            px, py = map(float, line.strip().split())
            # Convert pixel to map coordinate (origin + resolution * pixel)
            x = origin[0] + resolution * px
            y = origin[1] + resolution * (image_height - py)  # flip y-axis
            nodes.append({'id': idx, 'x': x, 'y': y})
    return nodes

def load_edges_from_txt(file_path):
    edges = []
    with open(file_path, 'r') as f:
        for line in f:
            i1, i2 = map(int, line.strip().split())
            edges.append((i1, i2))
    return edges

rospy.init_node('topological_map_marker_publisher')

# Publisher for nodes
node_pub = rospy.Publisher('/topo_nodes_marker', Marker, queue_size=10)
# Publisher for edges
edge_pub = rospy.Publisher('/topo_edges_marker', Marker, queue_size=10)

nodes = load_nodes_from_txt('/home/soon/map/topo_nodes.txt')
edges = load_edges_from_txt('/home/soon/map/topo_edges.txt')

# Prepare node marker (red spheres)
node_marker = Marker()
node_marker.header.frame_id = "map"
node_marker.ns = "topo_nodes"
node_marker.id = 0
node_marker.type = Marker.SPHERE_LIST
node_marker.action = Marker.ADD
node_marker.scale.x = 0.2
node_marker.scale.y = 0.2
node_marker.scale.z = 0.2
node_marker.color.a = 1.0
node_marker.color.r = 1.0
node_marker.color.g = 0.0
node_marker.color.b = 0.0

for node in nodes:
    p = Point()
    p.x = node["x"]
    p.y = node["y"]
    p.z = 0.0
    node_marker.points.append(p)

# Prepare edge marker (blue lines)
edge_marker = Marker()
edge_marker.header.frame_id = "map"
edge_marker.ns = "topo_edges"
edge_marker.id = 1
edge_marker.type = Marker.LINE_LIST
edge_marker.action = Marker.ADD
edge_marker.scale.x = 0.05  # line width
edge_marker.color.a = 1.0
edge_marker.color.r = 0.0
edge_marker.color.g = 0.0
edge_marker.color.b = 1.0

for i1, i2 in edges:
    p1 = Point()
    p2 = Point()
    p1.x = nodes[i1]["x"]
    p1.y = nodes[i1]["y"]
    p1.z = 0.0
    p2.x = nodes[i2]["x"]
    p2.y = nodes[i2]["y"]
    p2.z = 0.0
    edge_marker.points.append(p1)
    edge_marker.points.append(p2)

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    now = rospy.Time.now()
    node_marker.header.stamp = now
    edge_marker.header.stamp = now

    node_pub.publish(node_marker)
    edge_pub.publish(edge_marker)

    rate.sleep()

