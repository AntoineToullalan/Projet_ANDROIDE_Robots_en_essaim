from controller import Robot, DistanceSensor, Motor, Emitter, Receiver
from time import perf_counter as clock
import uuid, random
import time

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
TIME_STEP = 16

MAX_SPEED = 100
RANGE = (MAX_SPEED / 2)

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

mr, ml = 0, 0
l, r = 0, 0

speed = [0, 0]
braitenberg_coefficients =[ [10, 8], [7, -1.5], [5, -1], [-1, -1], [-1, -1], [-1, -1], [-1, 5], [-1.5, 7] ];

start_time = -1

old_back = False

MAX_COUNT = 5

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
    
    # r = psValues[1]*3 + psValues[2]*2 + psValues[3]
    # l = psValues[5] + psValues[6]*2 + psValues[7]*3
    right_obstacle = psValues[1] > 80.0 or psValues[2] > 80.0 or psValues[3] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    
    mr = max(mr, r)
    ml = max(ml, l)
    
    
    
    if(old_back and not back_obstacle):
        r = 0
        l = 0
    elif(psValues[1] > 100.0 ):
        r = 0
        l = MAX_SPEED
    elif(psValues[7] > 100.0):
        r = MAX_SPEED
        l = 0
    elif(front_obstacle):
        print("follow")
        r = MAX_SPEED
        l = MAX_SPEED
    else:
        for i in range(2):
          speed[i] = 0.0;
          for j in range(8):
            speed[i] += braitenberg_coefficients[j][i] * (1.0 - (psValues[j] / RANGE))
        r = min(speed[0], MAX_SPEED)
        l = min(speed[1], MAX_SPEED)
    
    
    leftMotor.setVelocity(l)
    rightMotor.setVelocity(r)
    
    old_back = back_obstacle
        