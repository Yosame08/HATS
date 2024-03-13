import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import chi2, norm
from math import exp, sqrt

granularity = 1
statBegin = 0
statEnd = 5000

# 你的数据
data = {}
with open("../train_turn_cnt.txt", "r") as f:
    inter = 0
    for line in f:
        info = line.split(' ')[:-1]
        if len(info) <= 3:
            inter = int(info[0])
            data[inter] = [0 for _ in range(int((statEnd - statBegin) * (1 / granularity)) + 1)]
            continue
        for i in range(len(info)):
            #val = float(info[i]) / inter / granularity
            val = float(info[i]) / granularity
            if val >= 0:
                data[inter][int(val)] += 1

for key in data:
    data[key] = np.array(data[key]) / sum(data[key])

# 创建x轴的值
x = np.arange(statBegin, statEnd + granularity, granularity)

chi10 = chi2.pdf(x/20, 10)/20
norm = norm.pdf(x, 5, 10)
beta = 2
y1 = np.sqrt(np.exp(-x / beta) / beta)
# for i in range(statBegin, statEnd+statInter, statInter):
#     y1.append((1/beta)*np.exp(-x/beta))

# 创建一个新的figure
plt.figure()
for key in data:
    val = key / 300
    plt.plot(x, data[key], marker='x', color=(val, 0, val), linewidth=1, markersize=2)
plt.plot(x, chi10, marker='+', color=(0, 0, 1), linewidth=1, markersize=0)
plt.plot(x, norm, marker='+', color=(0, 0, 1), linewidth=1, markersize=0)

plt.title('p')
plt.xlabel('Distance Difference')
plt.ylabel('Sum')
plt.xlim([0, 100])
plt.ylim([0, 0.5])

# 显示图形
plt.tight_layout()
plt.show()
