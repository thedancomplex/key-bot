import notes 
import numpy as np
import time
f4 = notes.getNote('F4')
g4 = notes.getNote('G4')
a5 = notes.getNote('A5')

print(f4)
print(g4)
print(a5)

note = 'F4'
notes.TORQUE = 0.0
notes.VELOS  = 60.0
notes.doOpen()


while True:
  x = notes.getPos()
  print(x)
  time.sleep(0.1)

