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

# feedback loop: step simulation until receiving an exit event
START = clock()
while robot.step(TIME_STEP) != -1:
       
       
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    if clock() - START > 1:
        START = clock()
        for el in psValues:
           print(round(el,3),end="       ")

    angle += 0.01

    # detect obstacles
    # ps0 = front
    # ps1 = front / right
    # ...
    # ps4 = back (not playstation 4)
    
    front_obstacle = psValues[0] > 80.0
    back_obstacle = psValues[4] > 80.0
    right_obstacle = psValues[1] > 80.0 or psValues[2] > 80.0 or psValues[3] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    
    if front_obstacle or back_obstacle or right_obstacle or left_obstacle:
        if front_obstacle:
            leftMotor.setVelocity(MAX_SPEED/2)
            rightMotor.setVelocity(-MAX_SPEED/2)
        elif back_obstacle:
            leftMotor.setVelocity(-MAX_SPEED/2)
            rightMotor.setVelocity(MAX_SPEED/2)
        elif right_obstacle:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(MAX_SPEED/2)
        elif left_obstacle:
            leftMotor.setVelocity(MAX_SPEED/2)
            rightMotor.setVelocity(0)
        robot.step(random.randint(TIME_STEP, TIME_STEP*10))
    else:
        leftMotor.setVelocity(MAX_SPEED/2)
        rightMotor.setVelocity(MAX_SPEED/2)
    
    
    