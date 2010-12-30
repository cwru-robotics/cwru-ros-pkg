#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import scipy as sp
import scipy.interpolate as interp

figure = plt.figure()
ax = figure.add_subplot(111)
lines = []

time_samples = np.linspace(0,1,100)

ys = [x*x*x - x*x + x for x in time_samples]
true_curve = ax.plot(time_samples, ys, 'r')
lines.append(true_curve[0])

sigma = 0.01
w = 1. / (np.ones(time_samples.shape) * sigma)

noisy_ys = [y + np.random.randn()*sigma for y in ys]
noisy_curve = ax.plot(time_samples, noisy_ys, 'g')
lines.append(noisy_curve[0])

spline = interp.UnivariateSpline(time_samples, noisy_ys,w=w)
spline_ys = [spline(x) for x in time_samples]
spline_curve = ax.plot(time_samples, spline_ys, 'b')
lines.append(spline_curve[0])

for line in lines:
    ax.add_line(line)

plt.legend(lines, ('True', 'Noisy', 'Spline'))
plt.grid()
plt.show()
