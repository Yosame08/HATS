import matplotlib.pyplot as plt
import numpy as np
from math import exp, sqrt

granularity = 1
statBegin = -200
statEnd = 6000

def phi(x):
    # constants
    a1 = 0.254829592
    a2 = -0.284496736
    a3 = 1.421413741
    a4 = -1.453152027
    a5 = 1.061405429
    p = 0.3275911

    # Save the sign of x
    sign = 1
    if x < 0:
        sign = -1
    x = abs(x) / sqrt(2.0)

    # A&S formula 7.1.26
    t = 1.0 / (1.0 + p * x)
    y = 1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * exp(-x * x)

    return 0.5 * (1.0 + sign * y)


def CalcArea(_mu, _sigma):
    return phi(_mu / _sigma)

# 你的数据
data = {}
with open("../valid_difDist_cnt.txt", "r") as f:
    inter = 0
    for line in f:
        info = line.split(' ')[:-1]
        if len(info) <= 3:
            inter = int(info[0])
            data[inter] = [0 for _ in range(int((statEnd-statBegin)*(1/granularity))+1)]
            continue
        for i in range(len(info)):
            data[inter][int((float(info[i])-statBegin)/granularity)] += 1

for key in data:
    data[key] = np.array(data[key]) / sum(data[key])

# 创建x轴的值
x = np.arange(statBegin, statEnd+granularity, granularity)

beta = 50
y2 = np.exp(-x/beta) / beta
y3 = 0.95/CalcArea(250,400)/sqrt(2*3.1415926)/400 * np.exp(-(x-250)**2/(2*400*400)) + 0.05/sqrt(2*3.1415926)/3 * np.exp(-x**2/(2*3*3))
# for i in range(statBegin, statEnd+statInter, statInter):
#     y1.append((1/beta)*np.exp(-x/beta))

# 创建一个新的figure
plt.figure()
for key in data:
    val = key/360
    plt.plot(x, data[key], marker='x', color=(val, 0, val), linewidth=1, markersize=3)
plt.plot(x, y3, marker='+', color=(0, 1, 0), linewidth=1, markersize=0)
plt.plot(x, y2, marker='+', color=(0, 0, 1), linewidth=1, markersize=0)

plt.title('p')
plt.xlabel('Distance Difference')
plt.ylabel('Sum')
plt.xlim([-5, 300])
plt.ylim([0, 0.15])

# 显示图形
plt.tight_layout()
plt.show()
