import notes 
import numpy as np
import time
f4 = notes.getNote('F4')
g4 = notes.getNote('G4')
a5 = notes.getNote('A5')

print(f4)
print(g4)
print(a5)

tn = ['E4', 'F4', 'G4', 'A5', 'B5', 'C5', 'D5', 'E5']

notes.TORQUE = 0.6
notes.VELOS  = 60.0
notes.doOpen()
time.sleep(0.5)
notes.setNoteUp('UP')
time.sleep(2.0)

for i in range(len(tn)):
  note = tn[i]
  print(note)
  notes.setNoteUp(note)
  time.sleep(0.1)
  notes.setNoteDown(note)
  time.sleep(0.1)
  notes.setNoteUp(note)
  time.sleep(0.1)

time.sleep(10.0)
notes.setNoteUp('UP')
time.sleep(2.0)
