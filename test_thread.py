import notes 
import numpy as np
import time

note = 'F4'
notes.TORQUE = 0.6
notes.VELOS  = 60.0
notes.doOpen()
while True:
  notes.kbc.theta[0] =  45.0
  print(notes.getPosJoints())
  time.sleep(2.0)
  notes.kbc.theta[0] = -45.0
  print(notes.getPosJoints())
  time.sleep(2.0)
