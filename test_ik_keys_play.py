import key_bot_arm_right as kbar
import key_bot_ik as ik
import key_bot_h as kb
import time as t
import copy 

kb.doOpen()

for i in range(len(kb.kbar.JOINTS)):
  the_id = kb.kbar.JOINTS[i]
  kb.torqueEnable(the_id)
  kb.setVelosDeg(the_id,50.0)
  kb.setTorque(the_id,0.50)

des3 = [0.1131434, -0.16205538, 0.07500772]
des4 = [des3[0], des3[1]+0.05, des3[2]]
#des4 = [des3[0], des3[1]+0.05, des3[2]+0.03]
#des4 = [des3[0], des3[1]+0.05, des3[2]-0.01]
#des4 = [0.0, 0.0, 0.4]


def setIK(val):
  d = [0.0, 0.0, 0.0, 0.0, 0.0]
  r = [0.0, 0.0, 0.0, 0.0, 0.0]
  for i in range(len(kb.kbar.JOINTS)):
    the_id = kb.kbar.JOINTS[i]
    deg = kb.getPosDeg(the_id)
    d[i] = deg
    r[i] = kb.deg2rad(deg)
  fk = ik.getFkArm(r)
  a = val
  order = ['p_x', 'p_y', 'p_z' ]
  tick = t.time()
  ik_theta, stat = ik.getIK(r,a, order) 
  tock = t.time()
  print(tock-tick)
  print(ik_theta)
  ids  = []
  degs = []
  for i in range(len(kb.kbar.JOINTS)):
    the_id = kb.kbar.JOINTS[i]
    ids.append(the_id)
    #deg = notes.C4[i]
    deg = kb.rad2deg(ik_theta[i])
    #kb.setPosDeg(the_id, deg)
    degs.append(deg)
  return (ids, degs)


OFFSET_UP   = 0.03
OFFSET_DOWN = -0.01
T = 1.0
OFFSET_SIDE = 0.75 * 0.0254
OFFSET_X    = 0.02
des4[1] = des4[1] + OFFSET_SIDE*0
while True:
  des = copy.deepcopy(des4)
  des[2] = des[2] + OFFSET_UP
  ids, degs = setIK(des)
  print(ids)
  print(degs)
  kb.setPosSyncDeg(ids, degs)
  t.sleep(T/2.0)

  des = copy.deepcopy(des4)
  des[2] = des[2] + OFFSET_DOWN
  ids, degs = setIK(des)
  kb.setPosSyncDeg(ids, degs)
  t.sleep(T/2.0)

