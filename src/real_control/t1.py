#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2022/7/23
# @Author : Zzy
import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import scipy.stats as st
matplotlib.rcParams.update({'font.size': 12})

# generate dataset
data_points = 50
sample_points = 10000
Mu = (np.linspace(-5, 5, num=data_points)) ** 2
Sigma = np.ones(data_points) * 8
data = np.random.normal(loc=Mu, scale=Sigma, size=(100, data_points))


# predicted expect and calculate confidence interval
predicted_expect = np.mean(data, 0)
low_CI_bound, high_CI_bound = st.t.interval(0.95, data_points - 1,
                                            loc=np.mean(data, 0),
                                            scale=st.sem(data))

# plot confidence interval
x = np.linspace(0, data_points - 1, num=data_points)
plt.plot(predicted_expect, linewidth=3., label='estimated value')
plt.plot(Mu, color='r', label='grand truth')
plt.fill_between(x, low_CI_bound, high_CI_bound, alpha=0.5,
                label='confidence interval')
plt.legend()
plt.title('Confidence interval')
plt.show()
