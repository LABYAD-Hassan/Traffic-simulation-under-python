import pygame
import sys
from importlib.machinery import SourceFileLoader

dsp = SourceFileLoader("traffic module","the file path").load_module()

red=255,0,0
purple=255,0,255
r1=dsp.Road([0,200],[400,200])
r3=dsp.Road([500,720],[500,300])
C=dsp.beziercurve([400,200],[500,300],[500,200])
pathaway=[r1]+C+[r3]
vehi1=dsp.Vehicle(pathaway,1,red,{"a0i":1.44, "bi":4.61, "v0i":16.6, "s0i":40,"Ti":1,"delta":4,"l":40})
vehi2=dsp.Vehicle(pathaway,20,purple,{"a0i":1.44, "bi":4.61, "v0i":32, "s0i":40,"Ti":1,"delta":4,"l":40})
sim=dsp.Simulation(0.05)
sim.create_roads(pathaway)
sim.input_vehicle(vehi1)
sim.input_vehicle(vehi2)
w=dsp.Window(sim)
w.loop()
