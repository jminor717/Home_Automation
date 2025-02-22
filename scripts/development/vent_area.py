import math
import matplotlib.pyplot as plt
import numpy as np


def remap(value, from_min, from_max, to_min, to_max):
  """
  Remaps a value from one range to another.

  Args:
    value: The input value to remap.
    from_min: The minimum value of the original range.
    from_max: The maximum value of the original range.
    to_min: The minimum value of the target range.
    to_max: The maximum value of the target range.

  Returns:
    The remapped value in the target range.
  """
  return (value - from_min) / (from_max - from_min) * (to_max - to_min) + to_min

pipeDiameter = 6
shaftDiameter = 0.25
alpha0 = 2 # fully closed angle to prevent wall binding

pipeArea = math.pi * ((pipeDiameter/2) ** 2)
x = []
y = []
y2 = []

D = pipeDiameter
b = shaftDiameter
pi = math.pi
cosA0 = math.cos(math.radians(alpha0))

print("find relation between commanded percent and actual percent")
# https://www.diva-portal.org/smash/get/diva2:23184/fulltext01
for aaa in range(2 , 100):
    alpha = remap(aaa, 0, 100, alpha0, 90)
    cosA = math.cos(math.radians(alpha))
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
    # print(alpha, (area / pipeArea) * 100)
    if area > 0:
        xx = (alpha / 91)
        yy = (area / pipeArea)
        print(f"{yy:.4f}, ")#{xx:.2f},
        # yy2 = (areaWShaft / pipeArea) * 100
        x.append(xx)
        y.append(yy)
        # y2.append(yy2)

print("find command required to achieve a given open percentage")

"""
area = (openPercent / 100) * pipeArea
cosA0 = math.cos(math.radians(alpha0))
cosA = -(area / ((pi * D**2)/4) - 1) * cosA0 

"""
cosA0 = math.cos(math.radians(alpha0))
for openPercent in range(0 , 100):
    area = (openPercent / 100) * pipeArea
    cosA = -((area / ((pi * D**2)/4)) - 1) * cosA0
    angle = math.degrees( math.acos(cosA))
    cmd = (angle / 90)
    # for a commanded airflow percentage, damper should be command to this percent open, resulting in an angle of
    # print(openPercent, ", ", cmd,", ", angle)
    print(f"{cmd:.4f}, ")

# plt.plot(x, y)
# # plt.plot(x, y2)
# plt.xlabel('damper angle')
# plt.ylabel('percent area open')
# plt.show()
