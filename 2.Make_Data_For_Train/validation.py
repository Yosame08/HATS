import matplotlib.pyplot as plt
import numpy as np
from math import exp, sqrt


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
    return S1 / CalcArea(0, sig1) / (sqrt_2_PI * sig1) * exp(-x2 / (a2 * 2)) + (1-S1) / CalcArea(mu2,sig2) / (sqrt_2_PI * sig2) * exp(-x_mu_2 / (b2 * 2))
        #S2 / CalcArea(mu2, sig2) / (sqrt_2_PI * sig2) * exp(-x_mu_2 / (b2 * 2)) + \
        #(1 - S1 - S2) / CalcArea(mu3, sig3) / (sqrt_2_PI * sig3) * exp(-x_mu_3 / (c2 * 2))

plt.figure()
granularity = 1
# 你的数据
data = []
with open("../ParamTurn.txt", "r") as f:
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
	    

# fit = [func(i) for i in range(len(data))]
# area = sum(fit)
# fit_fixed = np.array(fit) / area
# loss = 0
# for i in range(len(data)):
#     loss += abs(fit_fixed[i] - data[i])
# data = np.array(data)

# 创建x轴的值
# x = np.arange(0, len(data))

# plt.plot(x, fit_fixed, marker='+', color='b', linewidth=1, markersize=2)
plt.title('p')
plt.xlabel('Degree')
plt.ylabel('Sum')
plt.xlim([0, 1000])
plt.ylim([0, 0.05])

# 显示图形
plt.tight_layout()
plt.show()
