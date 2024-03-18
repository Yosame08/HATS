import copy
import numpy as np
import torch
import torch.nn as nn

input_size = 15
output_size = 1
model = nn.Sequential(
    nn.Linear(input_size, 512), nn.ELU(),
    nn.Linear(512, 256), nn.ELU(),
    nn.Linear(256, 128), nn.ELU(),
    nn.Linear(128, 64), nn.ELU(),
    nn.Linear(64, 32), nn.ELU(),
    nn.Linear(32, 16), nn.ELU(),
    nn.Linear(16, 8), nn.ELU(),
    nn.Linear(8, output_size),
)
model.load_state_dict(torch.load('model_vel.pth'))
model.eval()

roads = {}
with open("road_vectors.txt") as f:
    lines = f.readlines()[1:]
    for line in lines:
        info = line.split(' ')
        for i in range(len(info)):
            info[i] = float(info[i])
        roads[int(info[0])] = info[1:]


def pred(roadID, dist, prob, timestamp, journey, red):
    arr = copy.copy(roads[roadID])
    arr.append(dist)
    arr.append(prob)
    arr.append(timestamp)
    arr.append(journey)
    arr.append(red)
    with torch.no_grad():
        output = model(torch.from_numpy(np.array(arr)).float())
        print(output.item())


for i in range(0, 100, 10):
    pred(28616, 10.2804031, 0.187578964, 6.93444443, 0.62, i)
