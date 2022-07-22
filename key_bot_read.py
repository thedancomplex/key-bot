import key_bot_h as kb
import time as t

kb.doOpen()

for i in range(len(kb.kbar.JOINTS)):
  the_id = kb.kbar.JOINTS[i]
  kb.torqueEnable(the_id)
  kb.setVelosDeg(the_id,30.0)
  kb.setTorque(the_id,0.0)


deg = 10
while True:
  deg = 0.0
  for i in range(len(kb.kbar.JOINTS)):
    the_id = kb.kbar.JOINTS[i]
    kb.setPosDeg(the_id, deg)
    deg = kb.getPosDeg(the_id)
    print(deg, end='')
    print(", ", end='')
  print(" ")
  t.sleep(0.1)


