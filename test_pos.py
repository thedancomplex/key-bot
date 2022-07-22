import key_bot_h as kb
import time as t
import key_bot_arm_right_notes as notes

kb.doOpen()

for i in range(len(kb.kbar.JOINTS)):
  the_id = kb.kbar.JOINTS[i]
  kb.torqueEnable(the_id)
  kb.setVelosDeg(the_id,30.0)
  kb.setTorque(the_id,0.80)


while True:
  for i in range(len(kb.kbar.JOINTS)):
    the_id = kb.kbar.JOINTS[i]
    #deg = notes.C4[i]
    #kb.setPosDeg(the_id, deg)
    deg = kb.getPosDeg(the_id)
    print(deg, end='')
    print(", ", end='')
  print(" ")
  doUp = -doUp
  t.sleep(5.0)


