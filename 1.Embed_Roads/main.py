from curses.ascii import isdigit

from gensim.models import Word2Vec
import math


def calculate_distance(lat1, lon1, lat2, lon2):
    # 地球半径，单位为公里
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
    road_lengths = [0.0 for _ in range(lim + 1)]
    with open(filename, 'r') as file:
        for line in file:
            data = line.strip().split()
            out_deg[int(data[1])] += 1
            in_deg[int(data[2])] += 1
            num_points = int(data[3])
            total_length = 0.0
            for i in range(num_points - 1):
                lat1 = float(data[4 + i * 2])
                lon1 = float(data[5 + i * 2])
                lat2 = float(data[6 + i * 2])
                lon2 = float(data[7 + i * 2])
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


lim = 40960
occur = [0 for i in range(lim+1)]
in_deg = [0 for i in range(lim+1)]
out_deg = [0 for i in range(lim+1)]
to_road = [0 for i in range(lim+1)]
file_edge = '../edgeOSM_Porto.txt'
road_lengths = read_map_data(file_edge)
file_level = '../wayTypeOSM_Porto.txt'
road_level = read_road_level(file_level)
# print(road_lengths)

lines = []
# 读取道路序列数据
with open('../train_full_matched.txt', 'r') as f:
    for line in f:
        if isdigit(line[0]):
            lines.append(line)
lines = lines[1:]

# 将每一行的道路ID序列转化为列表
sequences = [line.strip().split() for line in lines]
for seq in sequences:
    for x in seq:
        occur[int(x)] += 1

vec_size = 8
# 使用Word2Vec训练
model = Word2Vec(sequences, vector_size=vec_size, window=3, min_count=1, workers=8, sg=0)

# 保存模型
model.save("road_embedding.model")

# 将每条道路的向量保存到文件中
with open('../road_vectors.txt', 'w') as f:
    f.write(f'{lim}\n')
    for road_id in range(lim + 1):
        if str(road_id) in model.wv:
            vector = model.wv[str(road_id)]
            out_str = f'{road_id} {" ".join(map(str, vector))} '
        else:
            out_str = f'{road_id} {" ".join(map(str, [0 for _ in range(vec_size)]))} '
        f.write(out_str+f'{occur[road_id]/len(sequences)*1000} {road_level[road_id]} '
                        f'{(in_deg[to_road[road_id]]+out_deg[to_road[road_id]])} {road_lengths[road_id]*1000}\n')
