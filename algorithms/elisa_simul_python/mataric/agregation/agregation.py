from controller import Robot, DistanceSensor, Motor, Emitter, Receiver
from time import perf_counter as clock
import uuid

###############################################
################### MAIN ######################
###############################################

id = uuid.uuid1()

# time in [ms] of a simulation step
TIME_STEP = 64

MAX_SPEED = 130

d_aggregate=100

# create the Robot instance.
robot = Robot()

emitter = robot.getDevice("myEmitter")
receiver = robot.getDevice("myReceiver")

receiver.enable(TIME_STEP)


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

def isRobotDetected(receiver):
    return receiver.getQueueLength() > 0


def NoAgentsInDistAggregat(psValues,d_aggregate):
    res=True
    for ps in psValues:
        if(ps>=d_aggregate):
            res=False
            break
    return res

def DirectionLocalCentroid(psValues):
    leftM=MAX_SPEED/2
    rightM=MAX_SPEED/2
    
    front_obstacle = psValues[0] > 80.0
    back_obstacle = psValues[4] > 80.0
    right_obstacle = psValues[1] > 80.0 or psValues[2] > 80.0 or psValues[3] > 80.0
    left_obstacle = psValues[5] > 80.0 or psValues[6] > 80.0 or psValues[7] > 80.0
    
    if front_obstacle:
        leftM=MAX_SPEED/2
        rightM=-MAX_SPEED/2
    elif back_obstacle:
        leftM=-MAX_SPEED/2
        rightM=MAX_SPEED/2
    elif right_obstacle:
        leftM=0
        rightM=MAX_SPEED/2
    elif left_obstacle:
        leftM=MAX_SPEED/2
        rightM=0
    else:
        leftM=MAX_SPEED/2
        rightM=MAX_SPEED/2        
    return (leftM,rightM)
        
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

send_last = 0
stopped = False
to_answer = False
aggregate = False
angle = 0


###### Initialisation GPS #######
GPS = robot.getDevice("gps")
GPS.enable(TIME_STEP)

###### Initialisation log #######
# myName = robot.getName()
# filename = "logs/" + "log_" + myName + ".txt"
# log = open(filename,"a")
# log.close()
# log = open(filename,"w")


# feedback loop: step simulation until receiving an exit event
START = clock()
step = 0
MAX_STEP = 5_000
while robot.step(TIME_STEP) != -1:
    agentNearby = isRobotDetected(receiver)
    
    msg = bytes("1", "utf-8")
    emitter.send(msg)
    
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())

    collisionDetected = not NoAgentsInDistAggregat(psValues, d_aggregate)

    left, right = DirectionLocalCentroid(psValues)
    
    #angle += 0.01

    # detect obstacles
    # ps0 = front
    # ps1 = front / right
    # ...
    # ps4 = back (not playstation 4)
    
    if agentNearby:
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)
        # print("Agent found")
    elif collisionDetected:
        leftMotor.setVelocity(left)
        rightMotor.setVelocity(right)
        # print("There's a wall")
    else:
        leftMotor.setVelocity(right)
        rightMotor.setVelocity(left)

        
    
    # if step == MAX_STEP:
    #     print("Stop logging")
    #     log.close()
    # elif step < MAX_STEP:
    #     log.write( str(GPS.getValues()) + "\n" ) 
    
    
    if step % 100 == 0 and step < MAX_STEP:
        print(step)
    step += 1
 
    
   