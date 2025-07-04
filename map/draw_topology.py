import matplotlib.pyplot as plt
import cv2

# Load map image (grayscale)
map_img = cv2.imread('warehouse.pgm', cv2.IMREAD_GRAYSCALE)

fig, ax = plt.subplots()
ax.imshow(map_img, cmap='gray')
ax.set_title('Step 1: Left-click to select nodes, Middle/Right-click to finish')

nodes = []
edges = []

# Step 1: Select nodes
print("Start selecting nodes: Left-click to add, Middle/Right-click to finish")
clicked_points = plt.ginput(n=-1, timeout=0)
nodes = list(clicked_points)
print(f"Number of nodes selected: {len(nodes)}")

# Visualize nodes
for n in nodes:
    ax.plot(n[0], n[1], 'ro')

plt.draw()
ax.set_title('Step 2: Left-click twice to connect edges, Middle/Right-click to finish')

# Step 2: Connect edges
while True:
    pts = plt.ginput(n=2, timeout=0)
    if len(pts) < 2:
        break

    # Find closest node index function
    def closest_node(point):
        dists = [((point[0]-x)**2 + (point[1]-y)**2)**0.5 for x,y in nodes]
        return dists.index(min(dists))

    i1 = closest_node(pts[0])
    i2 = closest_node(pts[1])
    edges.append((i1, i2))

    # Draw edge
    x_vals = [nodes[i1][0], nodes[i2][0]]
    y_vals = [nodes[i1][1], nodes[i2][1]]
    ax.plot(x_vals, y_vals, 'b-')
    plt.draw()

print(f"Number of edges created: {len(edges)}")

# Save to files
with open('topo_nodes.txt', 'w') as f:
    for x,y in nodes:
        f.write(f"{x} {y}\n")

with open('topo_edges.txt', 'w') as f:
    for i1,i2 in edges:
        f.write(f"{i1} {i2}\n")

print("Nodes and edges saved to topo_nodes.txt and topo_edges.txt")

plt.show()

