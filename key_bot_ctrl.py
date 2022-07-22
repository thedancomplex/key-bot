import key_bot_arm_right as kbar
import _thread
import time

TT = 0.05
theta = [ 0.0, 0.0, 0.0, 0.0, 0.0 ]
ids   = kbar.JOINTS
RUN = True
def ctrlLoop(T, kb):
  while RUN:
    print(kb.setPosSyncDeg(ids, theta))
    time.sleep(T)    


def doStartCtrl(T=TT, kb=None):
  if kb == None:
    return 1
  _thread.start_new_thread(ctrlLoop, (T,kb))
  return 0

