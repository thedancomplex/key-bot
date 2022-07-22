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
notes.TORQUE = 0.6
notes.VELOS  = 60.0
notes.doOpen()
time.sleep(0.5)
notes.setNoteUp('UP')
time.sleep(2.0)
notes.setNoteUp(note)
time.sleep(2.0)
notes.setNoteDown(note)
time.sleep(2.0)
notes.setNoteUp(note)
time.sleep(2.0)
notes.setNoteUp('UP')

