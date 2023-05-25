import numpy as np
from simulation import *

# Run simulation with ratios from 0 to 100%
# Take run with best desired yaw minus actual yaw

min_angle = -0.587965415313701
max_angle =  0.527998108411473
total_throttle = 10
steering_angle_resolution = 13
throttle_percent_resolution = 13
n_samples = 100

def generate():
  lut = []
  i = 0

  angle_step = (max_angle - min_angle) / steering_angle_resolution
  for angle in np.linspace(min_angle, max_angle, steering_angle_resolution):
    print('%3d' % i, end=' ')
    lut.append([])
    for throttle in np.linspace(0, 100, throttle_percent_resolution):
      min_diff = None
      l, r = None, None
      for ratio in np.linspace(0, 100, n_samples):
        tp, rp = throttle/100, ratio/100
        lin, rin = total_throttle*tp*rp, total_throttle*tp*(1-rp)
        ax, ay, wheel_velocity, yaw_rate, des_rate = simulate(1, 1, tp, rp, angle)
        diff = abs(des_rate[-1] - yaw_rate[-1])

        if (min_diff is None) or (diff < min_diff):
          min_diff = diff
          l, r = lin, rin
      lut[-1].append((l, r))
      print('.', end=' ', flush=True)
    i += 1
    print()

  return lut

def toCLUT(lut):
  s = '{\n'
  for i in lut:
    s += '\t{ '
    for j in i:
      s += '{ %f, %f, }, ' % (j[0], j[1])
    s += '}, \n'
  s += '};'
  return s

#lut = generate()
