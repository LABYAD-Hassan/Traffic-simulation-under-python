import pygame
import sys
from importlib.machinery import SourceFileLoader
import numpy as np
import matplotlib.pyplot as plt

dsp = SourceFileLoader("traffic module","the file path").load_module() #input the path of your traffic module

red=255,0,0
purple=255,0,255
r1=dsp.Road([-40,360],[100,360])
r2=dsp.Road([100,360],[1280,360])
pathaway=[r1,r2]
vehi1=dsp.Vehicle([r2],1,red,{"a0i":1.44, "bi":4.61, "v0i":0, "s0i":40,"Ti":1,"delta":4,"l":40})
vehi1.position=[1200,360]
vehi2=dsp.Vehicle(pathaway,2,purple,{"a0i":1.44, "bi":4.61, "v0i":32, "s0i":40,"Ti":1,"delta":4,"l":40})
vehi2.a=1.44
sim=dsp.Simulation(0.05)
sim.create_roads(pathaway)
sim.input_vehicle(vehi1)
sim.input_vehicle(vehi2)
w=dsp.Window(sim)
w.loop()

x = np.arange(0, 60.005, 0.05)
y = vehi2.P

plt.plot(x, y)
plt.plot(x, y)
plt.xlabel('t')
plt.ylabel('acceleration')
plt.title('acceleration de vehicle')

plt.show()
