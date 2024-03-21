import matplotlib.pyplot as plt
import numpy as np
from math import exp, sqrt

granularity = 1
statBegin = 0
statEnd = 5000


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
    return 1 - phi(0 - _mu / _sigma)


def func(x):
    global sig1,S1,sig2,mu2 # ,S2,sig3,mu3
    sqrt_2_PI = 2.506628274631
    x2 = x * x
    x_mu_2 = (x - mu2) * (x - mu2)
    #x_mu_3 = (x - mu3) * (x - mu3)
    a2, b2 = sig1 * sig1, sig2 * sig2
    return S1 / CalcArea(0, sig1) / (sqrt_2_PI * sig1) * exp(-x2 / (a2 * 2)) + \
            (1-S1) / CalcArea(mu2,sig2) / (sqrt_2_PI * sig2) * exp(-x_mu_2 / (b2 * 2))

plt.figure()
# 你的数据
data = []
with open("ParamTurn.txt", "r") as f:
    lines = f.readlines()
    for line in lines:
        info = line.split(' ')
        if len(info)<3:
            time = int(info[0])
            continue
        param = info[:-1]
        for i in range(len(param)):
            param[i] = float(param[i])
        # sig1,S1,sig2,mu2,S2,sig3,mu3 = param
        sig1,S1,sig2,mu2 = param
        x = []
        y = []
        i = 0
        while i <= 4000:
            x.append(i)
            i+=1
            y.append(func(i))
        plt.plot(x, y, marker='x', color=(time/360, 0, 0), linewidth=1, markersize=0)

data2 = {}
with open("train_turn_cnt.txt", "r") as f:
    inter = 0
    for line in f:
        info = line.split(' ')[:-1]
        if len(info) <= 3:
            inter = int(info[0])
            data2[inter] = [0 for _ in range(int((statEnd - statBegin) * (1 / granularity)) + 1)]
            continue
        for i in range(len(info)):
            val = float(info[i]) / granularity
            if val >= 0:
                data2[inter][int(val)] += 1
for key in data2:
    data2[key] = np.array(data2[key]) / sum(data2[key])

plt.title('p')
plt.xlabel('Degree')
plt.ylabel('Sum')
plt.xlim([0, 1000])
plt.ylim([0, 0.05])
x = np.arange(statBegin, statEnd + granularity, granularity)
for key in data2:
    val = key / 360
    plt.plot(x, data2[key], marker='x', color=(val, 0, val), linewidth=1, markersize=1)

# 显示图形
plt.tight_layout()
plt.show()
