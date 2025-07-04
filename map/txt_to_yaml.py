import yaml

# 맵 메타 정보 (필요시 조정)
resolution = 0.05
origin = [-29.993122, -10.065166]  # origin[x,y]
image_height = 796  # 예시, 실제 이미지 높이(px)로 맞춰주세요

def pixel_to_map_coord(px, py):
    # y 좌표 반전 + 원점/해상도 적용
    x = origin[0] + resolution * px
    y = origin[1] + resolution * (image_height - py)
    return x, y

# 1) 노드 로드
nodes = []
with open('topo_nodes.txt', 'r') as f:
    for line in f:
        px, py = map(float, line.strip().split())
        x, y = pixel_to_map_coord(px, py)
        nodes.append({'x': x, 'y': y})

# 2) 엣지 로드
edges = {}
with open('topo_edges.txt', 'r') as f:
    for line in f:
        i1, i2 = map(int, line.strip().split())
        edges.setdefault(i1, []).append(i2)
        edges.setdefault(i2, []).append(i1)  # 무방향 그래프라 양쪽 모두 추가

# 3) waypoints.yaml 구조 생성
waypoints = {}
for i, node in enumerate(nodes):
    wp_name = f"node_{i}"
    connected = [f"node_{j}" for j in edges.get(i, [])]
    waypoints[wp_name] = {
        'position': [node['x'], node['y'], 0.0],
        'edges': connected
    }

# 4) yaml 파일로 저장
with open('waypoints.yaml', 'w') as f:
    yaml.dump(waypoints, f, default_flow_style=False, sort_keys=True)

print("waypoints.yaml 생성 완료")

