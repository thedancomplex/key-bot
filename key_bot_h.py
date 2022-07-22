#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import numpy as np
import key_bot_arm_right as kbar

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_AX_TORQUE_MAX         = 34
ADDR_AX_MAX_VEL            = 32

# Data Byte Length
LEN_MX_GOAL_POSITION       = 2
LEN_MX_PRESENT_POSITION    = 2

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 7                 # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 10           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 500            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


VEL_MAX_TICK = 1023
VEL_MIN_TICK = 0

TICK_NUM = 1024
DEG_MAX  = 300.0
DEG_PER_TICK = DEG_MAX / float(TICK_NUM)
DEG_CENTER_POS = 0.0
TICK_CENTER_POS = int(TICK_NUM / 2.0 + DEG_CENTER_POS / DEG_PER_TICK)

if TICK_CENTER_POS > (TICK_NUM -1):
  TICK_CENTER_POS = (TICK_NUM - 1)
elif TICK_CENTER_POS < 0:
  TICK_CENTER_POS = 0


def tick2deg(tick=None):
  if (tick == None):
    return DEG_CENTER_POS

  if tick < 0:
    tick = 0
  elif tick > (TICK_NUM - 1):
    tick = (TICK_NUM - 1)

  deg = DEG_PER_TICK * tick - (DEG_MAX / 2.0)
  if deg > (DEG_MAX / 2.0):
    deg = (DEG_MAX / 2.0)
  elif deg < -(DEG_MAX / 2.0):
    deg = -(DEG_MAX / 2.0)

  deg = deg - DEG_CENTER_POS

  return deg

def tick2rad(tick=None):
  deg = tick2deg(tick)
  rad = deg / 180.0 * np.pi
  return rad
 

def deg2tick(deg=None):
  if deg == None:
    return TICK_CENTER_POS

  tick = (deg + DEG_CENTER_POS) / DEG_PER_TICK + TICK_NUM / 2.0
  
  tick = int(tick)

  if tick > (TICK_NUM - 1):
    tick = (TICK_NUM - 1)
  elif tick < 0:
    tick = 0

  return tick



  
portHandler = None
packetHandler = None 
groupSyncWrite = None
def doOpen():
  global portHandler
  global packetHandler
  global groupSyncWrite
# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
  groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)



# Open port
  if portHandler.openPort():
    print("Succeeded to open the port")
  else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()
  if portHandler == None:
    return 1
  if packetHandler == None:
    return 1
  return 0


# Set port baudrate
  if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
  else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

def torqueEnable(the_id=DXL_ID):
# Enable Dynamixel Torque
  dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, the_id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
  if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    return 1
  elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
    return 1
  else:
    print("Dynamixel has been successfully connected")
  return 0

def doExampleDeg():
  print(doOpen())
  print(torqueEnable(DXL_ID))
  print(setVelosDeg(DXL_ID, 100.0))
  print(setTorque(DXL_ID, 0.1))
  index = 0
  goal_pos = [ 5.0, -5.0]
  while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break
    # Change goal position
    print(setPosDeg(DXL_ID, goal_pos[index]))
    while 1:
        pos = getPosDeg(DXL_ID)
        print("[ID:%03d] SetPos:%03d - PresPos:%03d" % (DXL_ID, goal_pos[index], pos))
        if not abs(goal_pos[index] - pos) > 1:
            break
    if index == 0:
        index = 1
    else:
        index = 0
  doClose()

def doExample():
  print(doOpen())
  print(torqueEnable(DXL_ID))
  index = 0
  while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break
    # Change goal position
    print(setPosTick(DXL_ID, dxl_goal_position[index]))
    while 1:
        pos = getPosTick(DXL_ID)
        print("[ID:%03d] SetPos:%03d - PresPos:%03d" % (DXL_ID, dxl_goal_position[index], pos))
        if not abs(dxl_goal_position[index] - pos) > DXL_MOVING_STATUS_THRESHOLD:
            break
    if index == 0:
        index = 1
    else:
        index = 0
  doClose()

def setPosDeg(the_id=None, deg=None, the_dir=kbar.DIR ):
    if the_id == None:
      return 1
    if deg == None:
      return 1
    ii = kbar.JOINTS.index(the_id)
    do_dir = the_dir[ii]
    deg = deg * do_dir
    tick = deg2tick(deg)
    return setPosTick(the_id, tick)

def setPosRad(the_id=None, rad=None, the_dir=kbar.DIR ):
    if the_id == None:
      return 1
    if rad == None:
      return 1
    ii = kbar.JOINTS.index(the_id)
    do_dir = the_dir[ii]
    rad = rad * do_dir

    tick = rad2tick(rad)
    return setPosTick(the_id, tick)


def setPosTick(the_id=None, tick=None ):
    global packetHandler
    global portHandler
    if the_id == None:
      return 1
    if tick == None:
      return 1

    if tick > (TICK_NUM -1):
      tick = TICK_NUM - 1
    elif tick < 0:
      tick = 0


    # Write goal position
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, the_id, ADDR_MX_GOAL_POSITION, tick)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    return 0

VEL_RPM_MAX = 114.0
VEL_MAX_TRUE = 1023
VEL_MIN_TRUE = 0
VEL_RPM_PER_UNIT = VEL_RPM_MAX / float(VEL_MAX_TRUE)
VEL_DEG_PER_SEC_UNIT = VEL_RPM_PER_UNIT * 360.0 / 60.0 
VEL_DEG_MAX_TRUE = VEL_DEG_PER_SEC_UNIT * VEL_MAX_TRUE
VEL_DEG_MIN_TRUE = VEL_DEG_PER_SEC_UNIT * VEL_MIN_TRUE

def rad2deg(rad=None):
  if rad == None:
    return None
  return rad / np.pi * 180.0

def deg2rad(deg=None):
  if deg == None:
    return None
  return deg / 180.0 * np.pi

def setVelosRad(the_id=None, vel=None):
  if the_id == None:
    return 1
  if vel == None:
    return 1
  deg = rad2deg(vel)
  return setVelosDeg(the_id, deg)

def setVelosDeg(the_id=None, vel=None):
  if the_id == None:
    return 1
  if vel == None:
    return 1
 
  if vel > VEL_DEG_MAX_TRUE:
    vel = VEL_DEG_MAX_TRUE
  elif vel < VEL_DEG_MIN_TRUE:
    vel = VEL_DEG_MIN_TRUE

  tick = vel / VEL_DEG_PER_SEC_UNIT
  tick = int(tick)

  if tick > VEL_MAX_TRUE:
    tick = VEL_MAX_TRUE
  elif tick < VEL_MIN_TRUE:
    tick = VEL_MIN_TRUE

  return setVelosTick(the_id, tick)

 

def setVelosTick(the_id=None, vel=None):
  if the_id == None:
    return 1
  if vel == None:
    return 1

  if vel > VEL_MAX_TICK:
    vel = VEL_MAX_TICK
  if vel < VEL_MIN_TICK:
    vel = VEL_MIN_TICK

  dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, the_id, ADDR_AX_MAX_VEL, vel)
  if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
  elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
  return 0

TORQUE_MAX_TICK = 1023
TORQUE_MIN_TICK = 0



def setTorque(the_id=None, vel=None):
  if the_id == None:
    return 1
  if vel == None:
    return 1

  if vel > 1.0:
    vel = 1.0
  if vel < 0.0:
    vel = 0.0

  tick = vel * (TORQUE_MAX_TICK - TORQUE_MIN_TICK)
  tick = int(tick)

  if tick > TORQUE_MAX_TICK:
    tick = TORQUE_MAX_TICK
  if tick < TORQUE_MIN_TICK:
    tick = TORQUE_MIN_TICK

  dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, the_id, ADDR_AX_TORQUE_MAX, tick)
  if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
  elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
  return 0


def getPosDeg(the_id=None, the_dir=kbar.DIR):
        if the_id == None:
          return None
        ii = kbar.JOINTS.index(the_id)
        do_dir = the_dir[ii]
        tick = getPosTick(the_id)
        return tick2deg(tick) * do_dir
def getPosRad(the_id=None, the_dir=kbar.DIR):
        if the_id == None:
          return None
        ii = kbar.JOINTS.index(the_id)
        do_dir = the_dir[ii]
        tick = getPosTick(the_id)
        return tick2rad(tick) * do_dir

def getPosTick(the_id=None ):
        if the_id == None:
          return None

        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, the_id, ADDR_MX_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))


        return dxl_present_position



def torqueDisable(the_id=None):
    if the_id == None:
      return 1

    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, the_id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        return 1
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        return 1
    return 0

def doClose():
  global portHandler
  # Close port
  portHandler.closePort()

def setPosSyncDeg(ids=None, degs=None):
    if(ids == None):
      return 1
    if(degs == None):
      return 1
    if( len(ids) != len(degs) ):
      return 1

    tick = []
    for i in range(len(ids)):
      tick.append(deg2tick(degs[i]))
    
    return setPosSyncTick(ids, tick)

def setPosSyncRad(ids=None, degs=None):
    if(ids == None):
      return 1
    if(degs == None):
      return 1
    if( len(ids) != len(degs) ):
      return 1

    tick = []
    for i in range(len(ids)):
      tick.append(rad2tick(degs[i]))
    
    return setPosSyncTick(ids, tick)

def setPosSyncTick(ids=None, degs=None):
    if(ids == None):
      return 1
    if(degs == None):
      return 1
    if( len(ids) != len(degs) ):
      return 1
   
    for i in range(len(ids)):
      the_id  = ids[i]
      the_deg = degs[i]

      # Allocate goal position value into byte array
      param_goal_position = [
            DXL_LOBYTE(the_deg), 
            DXL_HIBYTE(the_deg) 
            ] 

      # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
      dxl_addparam_result = groupSyncWrite.addParam(the_id, param_goal_position)
      if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % the_id)
        return 1


    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        return 1

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()
    return 0

