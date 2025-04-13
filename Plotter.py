# -*- coding: utf-8 -*-
"""
Created on Thu Apr 10 04:53:24 2025

@author: siddh
"""

import pandas as pd
from matplotlib import pyplot as plt
import numpy as np

f=pd.read_csv("PID_LOG.log",delim_whitespace=True, header=None)

Timestep=f[7][1:].values
Current_time=f[8][1:].values
Positionx=f[9][1:].values
print(Positionx)

plt.plot(Timestep,Positionx)
plt.show()