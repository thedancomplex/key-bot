import key_bot_arm_right as kbar
import key_bot_ik as ik
import key_bot_h as kb
import time as t
import copy
import key_bot_ctrl as kbc



NOTE_NAME = 0
NOTE_POS  = 1
NOTE_UP   = 2
NOTE_DOWN = 3
NOTE_X = 0
NOTE_Y = 1
NOTE_Z = 2


VELOS  = 50.0
TORQUE =  0.50


def getPosDeg(the_id=None):
  if the_id == None:
    return None
  try:
    ids = kbc.state_ids.index(the_id)
    st = getPosJoints()
    return st[ids]
  except:
    return None
  return None

def getPosRad(the_id=None):
  if the_id == None:
    return None
  
  deg = getPosDeg(the_id)
  rad = kb.deg2rad(deg)
  return rad
  


def getPosJoints():
  the_out = []
  the_out = kbc.state
#  for i in range(len(kb.kbar.JOINTS)):
#    the_id = kb.kbar.JOINTS[i]
#    deg = kb.getPosRad(the_id)
#    the_out.append(deg)
  return the_out

def getPos():
#  A = ik.getA(val)
  val = getPosJoints()
  A = ik.getFkArm(val)
  x = A[0,3]
  y = A[1,3]
  z = A[2,3]
  p = [ x, y, z ]
  return p

def doOpen(T=kbc.TT):
  kb.doOpen()

  for i in range(len(kb.kbar.JOINTS)):
    the_id = kb.kbar.JOINTS[i]
    kb.torqueEnable(the_id)
    kb.setVelosDeg(the_id,VELOS)
    kb.setTorque(the_id,TORQUE)

  return kbc.doStartCtrl(T, kb)



def setIK(val):
  d = [0.0, 0.0, 0.0, 0.0, 0.0]
  r = [0.0, 0.0, 0.0, 0.0, 0.0]
  for i in range(len(kb.kbar.JOINTS)):
    the_id = kb.kbar.JOINTS[i]
    deg = getPosDeg(the_id)
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





def addNote(notes=[], name=None, pos=None, pos_up=None, pos_down=None):
  if name == None:
    return 1
  if pos == None:
    return 1
  if pos_up == None:
    return 1
  if pos_down == None:
    return 1
  note      = []
  cname     = copy.deepcopy(name)
  cpos      = copy.deepcopy(pos)
  cpos_up   = copy.deepcopy(pos_up)
  cpos_down = copy.deepcopy(pos_down)
  note.append(cname)
  note.append(cpos)
  note.append(cpos_up)
  note.append(cpos_down)

  notes.append(note)
  return notes

def setNoteUp(name=None):
  if name == None:
    return 1

  val = getNoteUp(name)
  if val == 1:
    return 1

  ids, degs = setIK(val)

  kbc.theta = degs
  kbc.ids   = ids 
  return 0
  #return kb.setPosSyncDeg(ids, degs)

def setNoteDown(name=None):
  if name == None:
    return 1

  val = getNoteDown(name)
  if val == 1:
    return 1

  ids, degs = setIK(val)
  kbc.theta = degs
  kbc.ids   = ids 
  return 0
  #return kb.setPosSyncDeg(ids, degs)

def getNoteUp(name=None):
  if name == None:
    return 1
  val = getNote(name)

  if val == 1:
    return 1

  pos = val[NOTE_POS]
  pos[NOTE_Z] = pos[NOTE_Z] + val[NOTE_UP]
  return pos

def getNoteDown(name=None):
  if name == None:
    return 1
  val = getNote(name)

  if val == 1:
    return 1

  pos = val[NOTE_POS]
  pos[NOTE_Z] = pos[NOTE_Z] + val[NOTE_DOWN]
  return pos

def getNote(name=None):
  if name == None:
    return 1

  for i in range(len(notes)):
    try:
      val = notes[i]
      a = val.index(name)
      if a == NOTE_NAME:
        return val
    except:
      pass
  return 1
    

notes = []
notes = addNote( notes , 'UP', [ 0.0, 0.0, 0.4], 0.0, 0.0 )


E4_POS = [0.1087854110279702, -0.12875151458441686, 0.0785395050070696]
F4_POS = [0.09449643711508654, -0.10512175090980098, 0.06721989461110456]
G4_POS = [0.08084612098943962, -0.07792320175266998, 0.05301969529617176]
A5_POS = [0.07489533442285769, -0.06185051762398291, 0.04557337687149318]
B5_POS = [0.075394440198049, -0.04806769522583378, 0.04851158383899397]
C5_POS = [0.06672520004447989, -0.03392945445638974, 0.04747836898712067]
D5_POS = [0.06414436257324459, -0.02140892655152586, 0.04848650624678778]
E5_POS = [0.06210780194098414, -0.00960265585515535, 0.04867928264048951]


# E4
pos = E4_POS
pos[1] = pos[1] + -0.005
up = 0.03
down = -0.05
name = 'E4'
notes = addNote( notes, name, pos, up, down )

# F4
pos = F4_POS
up = 0.03
down = -0.045
name = 'F4'
notes = addNote( notes, name, pos, up, down )

# G4
pos = G4_POS
pos[1] = pos[1] + -0.003
pos[0] = pos[0] + 0.008
up = 0.03
down = -0.04
name = 'G4'
notes = addNote( notes, name, pos, up, down )


# A5
pos = A5_POS
pos[0] = pos[0] +0.008
pos[1] = pos[1] -0.005
up = 0.05
down = -0.055
name = 'A5'
notes = addNote( notes, name, pos, up, down )


# B5
pos = B5_POS
pos[0] = pos[0] + 0.008
pos[1] = pos[1] + -0.002
up = 0.05
down = -0.06
name = 'B5'
notes = addNote( notes, name, pos, up, down )

# C5
pos = C5_POS
pos[0] = pos[0] + 0.012
pos[1] = pos[1] + -0.002
up = 0.05
down = -0.06
name = 'C5'
notes = addNote( notes, name, pos, up, down )

# D5
pos = D5_POS
pos[0] = pos[0] + 0.012
up = 0.05
down = -0.06
name = 'D5'
notes = addNote( notes, name, pos, up, down )

# E5
pos = E5_POS
pos[0] = pos[0] + 0.012
up = 0.05
down = -0.06
name = 'E5'
notes = addNote( notes, name, pos, up, down )




C4  = [-55.37109375, 87.01171875, 30.76171875, -27.83203125, -31.0546875]
C4S = [-59.47265625, 72.94921875, 31.640625, 6.15234375, -34.5703125]
D4  = [-52.734375, 75.5859375, 31.34765625, 5.2734375, -38.96484375]
E4F = [-56.25, 68.84765625, 32.2265625, 15.52734375, -24.90234375]
E4  = [-50.390625, 72.36328125, 31.0546875, 13.4765625, -31.0546875]
F4  = [-45.703125, 62.6953125, 38.37890625, 30.76171875, -48.6328125]
F4S = [-50.68359375, 53.61328125, 43.359375, 53.61328125, -67.3828125]
G4  = [-41.6015625, 57.421875, 42.48046875, 45.703125, -62.6953125]
G4S = [-46.2890625, 51.5625, 46.2890625, 64.16015625, -75.29296875]
A5  = [-37.20703125, 57.421875, 39.2578125, 54.4921875, -65.625]
B5F = [-39.55078125, 46.2890625, 53.61328125, 70.3125, -83.49609375]
B5  = [-31.640625, 54.78515625, 42.7734375, 62.40234375, -73.828125]
C5  = [-24.90234375, 53.02734375, 45.41015625, 70.01953125, -85.546875]
C5S = [-26.66015625, 46.2890625, 55.6640625, 83.203125, -99.31640625]
D5  = [-17.87109375, 52.1484375, 49.21875, 75.87890625, -98.73046875]
E5F = [-15.234375, 49.51171875, 54.78515625, 92.578125, -113.96484375]
E5  = [-8.49609375, 55.6640625, 43.65234375, 85.546875, -106.640625]
F5  = [0.5859375, 55.95703125, 43.65234375, 86.71875, -106.640625]
F5S = [5.56640625, 55.37109375, 50.9765625, 101.07421875, -127.1484375]
G5  = [9.375, 60.9375, 33.10546875, 90.52734375, -102.24609375]
G5S = [20.21484375, 59.47265625, 36.328125, 103.41796875, -112.5]
A6  = [20.21484375, 62.6953125, 28.41796875, 89.6484375, -96.97265625]
B6F = [29.8828125, 60.9375, 30.17578125, 97.55859375, -98.4375]
B6  = [26.66015625, 63.8671875, 26.953125, 89.0625, -98.4375]
C6  = [32.51953125, 65.625, 19.921875, 76.46484375, -77.34375]


xyz     = [0.1131434, -0.16205538, 0.07500772]
dy      = 0.01

C4_pos  = [xyz[0], xyz[1], xyz[2]]
C4S_pos = [xyz[0], xyz[1], xyz[2]]
D4_pos  = [xyz[0], xyz[1], xyz[2]]
E4F_pos = [xyz[0], xyz[1], xyz[2]]
E4_pos  = [xyz[0], xyz[1], xyz[2]]
F4_pos  = [xyz[0], xyz[1], xyz[2]]
F4S_pos = [xyz[0], xyz[1], xyz[2]]
G4_pos  = [xyz[0], xyz[1], xyz[2]]
G4S_pos = [xyz[0], xyz[1], xyz[2]]
A5_pos  = [xyz[0], xyz[1], xyz[2]]
B5F_pos = [xyz[0], xyz[1], xyz[2]]
B5_pos  = [xyz[0], xyz[1], xyz[2]]
C5_pos  = [xyz[0], xyz[1], xyz[2]]
C5S_pos = [xyz[0], xyz[1], xyz[2]]
D5_pos  = [xyz[0], xyz[1], xyz[2]]
E5F_pos = [xyz[0], xyz[1], xyz[2]]
E5_pos  = [xyz[0], xyz[1], xyz[2]]
F5_pos  = [xyz[0], xyz[1], xyz[2]]
F5S_pos = [xyz[0], xyz[1], xyz[2]]
G5_pos  = [xyz[0], xyz[1], xyz[2]]
G5S_pos = [xyz[0], xyz[1], xyz[2]]
A6_pos  = [xyz[0], xyz[1], xyz[2]]
B6F_pos = [xyz[0], xyz[1], xyz[2]]
B6_pos  = [xyz[0], xyz[1], xyz[2]]
C6_pos  = [xyz[0], xyz[1], xyz[2]]

POS = [C4_pos, C4S_pos, D4_pos, E4F_pos, E4_pos, F4_pos, F4S_pos, G4_pos, G4S_pos, A5_pos, B5F_pos, B5_pos, 
         C5_pos, C5S_pos, D5_pos, E5F_pos, E5_pos, F5_pos, F5S_pos, G5_pos, G5S_pos, A6_pos, B6F_pos, B6_pos,
         C6_pos]
