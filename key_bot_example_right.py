import key_bot_h as kb
import time as t

kb.doOpen()

for i in range(len(kb.kbar.JOINTS)):
  the_id = kb.kbar.JOINTS[i]
  kb.torqueEnable(the_id)
  kb.setVelosDeg(the_id,30.0)
  kb.setTorque(the_id,0.5)


deg = 10
while True:
  deg = -deg
  for i in range(len(kb.kbar.JOINTS)):
    the_id = kb.kbar.JOINTS[i]
    kb.setPosDeg(the_id, deg)
  t.sleep(5.0)


