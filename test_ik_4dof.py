import key_bot_arm_right as kbar
import key_bot_ik as ik
import key_bot_h as kb
import time as t
import numpy as np
kb.doOpen()

for i in range(len(kb.kbar.JOINTS)):
  the_id = kb.kbar.JOINTS[i]
  kb.torqueEnable(the_id)
  kb.setVelosDeg(the_id,30.0)
  kb.setTorque(the_id,0.50)



dd = 0.10
de = 0.20
ds = 1.0

x = kbar.TOP_ROT_TO_ROT_X + kbar.EEF_OFFSET_X
y = kbar.BAS_OFFSET_Y
z = kbar.BAS_OFFSET_Z + kbar.TOP_ROT_TO_ROT_Z + kbar.ROT_TO_ROT + kbar.ROT_TO_ROT + kbar.EEF_OFFSET_Z
des = [x, y, z]

x1 = kbar.TOP_ROT_TO_ROT_X - kbar.EEF_OFFSET_Z
y1 = kbar.BAS_OFFSET_Y
z1 = kbar.BAS_OFFSET_Z + kbar.TOP_ROT_TO_ROT_Z + kbar.ROT_TO_ROT + kbar.ROT_TO_ROT + kbar.EEF_OFFSET_X
des1 = [x1, y1, z1]
des2 = [0.1, 0.0, 0.15]
des3 = [0.1131434, -0.16205538, 0.07500772]
des4 = [des3[0]+0.01, des3[1]+0.01, des3[2]+0.01, np.pi/2.0]
while True:
  d = [0.0, 0.0, 0.0, 0.0, 0.0]
  ddd = [0.0, 0.0, 0.0, 0.0, -90.0]
  r = [0.0, 0.0, 0.0, 0.0, 0.0]
  for i in range(len(kb.kbar.JOINTS)):
    the_id = kb.kbar.JOINTS[i]
    #deg = notes.C4[i]
    #kb.setPosDeg(the_id, deg)
    deg = kb.getPosDeg(the_id)
    d[i] = deg
    r[i] = kb.deg2rad(deg)
  fk = ik.getFkArm(r)
  ds = -ds
  df = de + dd * ds
  a = [ 0.15, 0.000, df]
  a = des4
  #ik_theta = ik.getIK3dof(r,a) 
  order = ['p_x', 'p_y', 'p_z', 't_x' ]
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
    degs.append(deg)

  kb.setPosSyncDeg(ids, degs)
  print(fk)
  print(ik_theta)
  print(des)
  print(des1)
  t.sleep(0.005)

