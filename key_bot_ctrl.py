import key_bot_arm_right as kbar
import _thread
import time
import copy

TT = 0.05
theta     = [ 0.0, 0.0, 0.0, 0.0, 0.0 ]
ids       = copy.deepcopy(kbar.JOINTS)
state     = copy.deepcopy(theta)
state_ids = copy.deepcopy(kbar.JOINTS)

RUN = True
def ctrlLoop(T, kb):
  global state
  global state_ids
  while RUN:
    kb.setPosSyncDeg(ids, theta)
    the_out = []
    the_ids = []
    for i in range(len(kb.kbar.JOINTS)):
      the_id = kb.kbar.JOINTS[i]
      the_ids.append(the_id)
      deg = kb.getPosRad(the_id)
      the_out.append(deg)
    state = the_out
    state_ids = the_ids
    time.sleep(T)    


def doStartCtrl(T=TT, kb=None):
  if kb == None:
    return 1
  _thread.start_new_thread(ctrlLoop, (T,kb))
  return 0

