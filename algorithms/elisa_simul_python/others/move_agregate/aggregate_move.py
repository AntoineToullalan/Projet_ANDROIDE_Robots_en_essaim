from controller import Robot, DistanceSensor, Motor, Emitter, Receiver
from time import perf_counter as clock
import uuid, random

def position_front(left, right, back):
    left_speed = 0.5 * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED
    if left or back:
        # turn left
        left_speed  = -0.5 * MAX_SPEED
        right_speed = 0.5 * MAX_SPEED
    else:
        # turn right
        left_speed  = 0.5 * MAX_SPEED
        right_speed = -0.5 * MAX_SPEED
    return left_speed, right_speed
    

def leave_wall(left, right, front, angle):
    left_speed = (angle%0.5) * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED
    if left or front:
        # turn right
        left_speed  = 0.5 * MAX_SPEED
        right_speed = -0.5 * MAX_SPEED
    elif right:
        # turn left
        left_speed  = -0.5 * MAX_SPEED
        right_speed = 0.5 * MAX_SPEED
    return left_speed, right_speed
    


###############################################
################### MAIN ######################
###############################################

id = uuid.uuid1()

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 20

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

old_front_obstacle = False
old_back_obstacle = False
old_right_obstacle = False
old_left_obstacle = False

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
    
    if(old_front_obstacle and not(front_obstacle)):
        l=MAX_SPEED/2
        r=MAX_SPEED/2
    elif(old_back_obstacle and not(back_obstacle)):
        l=-MAX_SPEED/2
        r=-MAX_SPEED/2
    elif(old_right_obstacle and not(right_obstacle)):
        l=MAX_SPEED/2
        r=0
    elif(old_left_obstacle and not(left_obstacle)):
        l=0
        r=MAX_SPEED/2
    else:
        l=MAX_SPEED/4
        r=MAX_SPEED/4
    
    
    leftMotor.setVelocity(l)
    rightMotor.setVelocity(r)
    
    
    old_front_obstacle = front_obstacle
    old_back_obstacle = back_obstacle
    old_right_obstacle = right_obstacle
    old_left_obstacle = left_obstacle
    
    
    