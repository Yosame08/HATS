from curses.ascii import isdigit
from gensim.models import Word2Vec
import math

lim = 0
nodeID = 262144
roadID = 262144

def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371.0
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    dlon = lon2_rad - lon1_rad
    dlat = lat2_rad - lat1_rad

    a = math.sin(dlat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c
    return distance


def read_map_data(filename):
    global lim
    road_lengths = [0.0 for _ in range(roadID)]
    with open(filename, 'r') as file:
        for line in file:
            data = line.strip().split()
            rid = int(data[0])
            if rid > lim:
                lim = rid
            out_deg[int(data[1])] += 1
            in_deg[int(data[2])] += 1
            num_points = int(data[3])
            total_length = 0.0
            for i in range(num_points - 1):
                lat1, lon1 = float(data[4 + i * 2]), float(data[5 + i * 2])
                lat2, lon2 = float(data[6 + i * 2]), float(data[7 + i * 2])
                total_length += calculate_distance(lat1, lon1, lat2, lon2)
            road_lengths[int(data[0])] = total_length
            to_road[int(data[0])] = int(data[2])
    return road_lengths


def read_road_level(filename):
    road_levels = [0 for _ in range(lim + 1)]
    with open(filename, 'r') as file:
        for line in file:
            data = line.strip().split()
            level = int(data[2])
            if level > 7 or level < 1:
                level = 1
            road_levels[int(data[0])] = level
    return road_levels


in_deg = [0 for i in range(nodeID)]
out_deg = [0 for i in range(nodeID)]
to_road = [0 for i in range(nodeID)]
occur = [0 for i in range(roadID)]
file_edge = '../Map/edgeOSM.txt'
road_lengths = read_map_data(file_edge)
file_level = '../Map/wayTypeOSM.txt'
road_level = read_road_level(file_level)
# print(road_lengths)

lines = []
# 读取道路序列数据
with open('../Intermediate/train_full.txt', 'r') as f:
    for line in f:
        if isdigit(line[0]):
            lines.append(line)
lines = lines[1:]

# 将每一行的道路ID序列转化为列表
sequences = [line.strip().split() for line in lines]
for seq in sequences:
    vis = [False for i in range(lim+1)]
    for x in seq:
        if vis[int(x)]:
            continue
        vis[int(x)] = True
        occur[int(x)] += 1

vec_size = 12
# 使用Word2Vec训练
model = Word2Vec(sequences, vector_size=vec_size, window=2, min_count=1, workers=8, sg=0)

# 将每条道路的向量保存到文件中
with open('../Intermediate/road_vectors.txt', 'w') as f:
    f.write(f'{lim}\n')
    for road_id in range(lim + 1):
        if str(road_id) in model.wv:
            vector = model.wv[str(road_id)]
            out_str = f'{road_id} {" ".join(map(str, vector))} '
        else:
            out_str = f'{road_id} {" ".join(map(str, [0 for _ in range(vec_size)]))} '
        f.write(out_str+f'{occur[road_id]/len(sequences)*100} {road_level[road_id]} '
                        f'{road_lengths[road_id]} {out_deg[to_road[road_id]]}\n')
