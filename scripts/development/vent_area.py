import math
import matplotlib.pyplot as plt
import numpy as np

pipeDiameter = 6
shaftDiameter = 0.25
alpha0 = 2 # fully closed angle to prevent wall binding

pipeArea = math.pi * ((pipeDiameter/2) ** 2)
x = []
y = []
y2 = []

# https://www.diva-portal.org/smash/get/diva2:23184/fulltext01
for alpha in range(alpha0 , 90):
    D = pipeDiameter
    b = shaftDiameter
    pi = math.pi
    cosA = math.cos(math.radians(alpha))
    cosA0 = math.cos(math.radians(alpha0))
    area = ((pi * D**2)/4) * (1 - (cosA / cosA0 ) )
    # areaWShaft = ((pi * D**2)/4) \
    #     * ( \
    #         (1 - (cosA / cosA0 ) ) \
    #         + ((2/pi) * ( \
    #             ((b/cosA) * ((math.cos(cosA)) - (b**2 * math.cos(cosA0)))**0.5) \
    #             + ( (cosA/cosA0) * (np.arcsin((b * cosA0)/cosA)) ) \
    #             - (b * (1-b**2)**0.5 ) \
    #             - (np.arcsin(b)) \
    #             ) \
    #         ) \
    #     )
    # print(round(area, 2) ,round(areaWShaft, 2),(b * cosA0)/cosA )
    print(alpha, (area / pipeArea) * 100)
    if area > 0:
        xx = (alpha / 90) * 100
        yy = (area / pipeArea) * 100
        # yy2 = (areaWShaft / pipeArea) * 100
        x.append(xx)
        y.append(yy)
        # y2.append(yy2)

plt.plot(x, y)
# plt.plot(x, y2)
plt.xlabel('damper angle')
plt.ylabel('percent area open')
plt.show()
