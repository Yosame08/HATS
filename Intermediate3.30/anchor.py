import matplotlib.pyplot as plt
import numpy as np
from math import exp, sqrt

granularity = 1
statBegin = 0
statEnd = 200

alls = []
data2 = {0: [0 for _ in range(int((statEnd - statBegin) * (1 / granularity)) + 1)]}
with open("anchorError.txt", "r") as f:
    inter = 0
    for line in f:
        info = line.split(' ')[:-1]
        if len(info) <= 3:
            inter = int(info[0])
            data2[inter] = [0 for _ in range(int((statEnd - statBegin) * (1 / granularity)) + 1)]
            continue
        for i in range(len(info)):
            val = float(info[i]) / granularity
            alls.append(val)
            if val >= 0:
                data2[inter][int(val)] += 1
print(np.median(np.array(alls)))
for key in data2:
    data2[key] = np.array(data2[key]) / sum(data2[key])

plt.title('p')
plt.xlabel('Degree')
plt.ylabel('Sum')
plt.xlim([0, statEnd])
plt.ylim([0, 0.2])
x = np.arange(statBegin, statEnd + granularity, granularity)
for key in data2:
    plt.plot(x, data2[key], marker='x', color=(1, 0, 1), linewidth=1, markersize=1)

# 显示图形
plt.tight_layout()
plt.show()
