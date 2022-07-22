import key_bot_h as kb
import time as t
import key_bot_arm_right_notes as notes

kb.doOpen()

for i in range(len(kb.kbar.JOINTS)):
  the_id = kb.kbar.JOINTS[i]
  kb.torqueEnable(the_id)
  kb.setVelosDeg(the_id,50.0)
  kb.setTorque(the_id,0.80)


doUp = 1
note_i = 0
note_i_max = len(notes.NOTES)-1

while True:
  deg = 0.0
  for i in range(len(kb.kbar.JOINTS)):
    off = 0.0
    #if ( (i != 0) & (i != (len(kb.kbar.JOINTS) - 1)) ) :
    doUp = 1
    if(doUp == 1):
      if ( i == 1 ) :
        off = -50.0
      if (i == 3):
        off = 25.0
    note = notes.NOTES[note_i]      
    the_id = kb.kbar.JOINTS[i]
    deg = note[i] + off
    kb.setPosDeg(the_id, deg)
    deg = kb.getPosDeg(the_id)
    print(deg, end='')
    print(", ", end='')
  print(" ")
  doUp = -doUp
  note_i = note_i + 1
  if note_i > note_i_max:
    note_i = 0
  t.sleep(1.0)


