import math
import matplotlib.pyplot as plt
import numpy as np

data = []
with open("../valid_turn_cnt.txt") as f:
    info = f.readline().split(' ')[:-1]
    for i in info:
        data.append(float(i))
data.sort()

x = [0]
y = [1]
tot = len(data)
now = tot
for i in range(tot):
    if not i == tot - 1 and data[i + 1] == data[i]:
        continue
    now = tot - i - 1
    if not i == tot - 1:
        x.append(data[i + 1])
        y.append(now / tot)

p1, p2 = 1, 30
x2 = np.arange(0, 180, 0.1)
y2 = 1 / x2
y3 = 5 / (math.sqrt(math.pi * 2) * p2) * np.exp(-(x2 - 60) ** 2 / (p2 ** 2 * 2))

plt.figure()
plt.plot(x, y, marker='x', color='r', linewidth=1, markersize=1)
plt.plot(x2, y2, marker='x', color='g', linewidth=1, markersize=1)
plt.plot(x2, y3, marker='x', color='g', linewidth=1, markersize=1)
plt.plot(x2, y2 + y3, marker='x', color='b', linewidth=1, markersize=1)
plt.title('P(X>=x)')
plt.xlabel('Degree')
plt.ylabel('Probability')
plt.xlim([0, 180])
plt.ylim([0, 1])
plt.tight_layout()
plt.show()
