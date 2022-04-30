from controller import Robot, DistanceSensor, Motor, Emitter, Receiver
from time import perf_counter as clock
import uuid, random

###############################################
################### MAIN ######################
###############################################

id = uuid.uuid1()

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 130

# create the Robot instance.
robot = Robot()

# initialize devices
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

ds = []

dsNames = [
    'fs0', 'fs1', 'fs2', 'fs3'
]

for i in range(len(psNames)):
    if i < len(dsNames):
        ds.append(robot.getDevice(dsNames[i]))
        ds[i].enable(TIME_STEP)
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)
    

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(100)
rightMotor.setVelocity(100)
send_last = 0
stopped = False
to_answer = False
aggregate = False
angle = 0

mr, ml = 0,0

# feedback loop: step simulation until receiving an exit event
START = clock()
while robot.step(TIME_STEP) != -1:
       
       
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    angle += 0.01

    # detect obstacles
    # ps0 = front
    # ps1 = front / right
    # ...
    # ps4 = back (not playstation 4)
    
    front_obstacle = psValues[0] > 80.0
    back_obstacle = psValues[4] > 80.0
    
    r = psValues[1]*3 + psValues[2]*2 + psValues[3]
    l = psValues[5] + psValues[6]*2 + psValues[7]*3
    right_obstacle = psValues[1] > 80.0 or psValues[2] > 80.0 or psValues[3] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    
    mr = max(mr, r)
    ml = max(ml, l)
    
    if mr > MAX_SPEED:
        mr = MAX_SPEED
    elif ml > MAX_SPEED:
        ml = MAX_SPEED
    
    if front_obstacle:
        dir = -1
    else:
        dir = 1
    
    r = r%mr * dir
    l = l%ml * dir
    
    if r > MAX_SPEED:
        r = MAX_SPEED
    elif l > MAX_SPEED:
        l = MAX_SPEED
    elif l < -MAX_SPEED:
        l = -MAX_SPEED
    elif r < -MAX_SPEED:
        r = -MAX_SPEED
    
    leftMotor.setVelocity(l)
    rightMotor.setVelocity(r)
    
    
    