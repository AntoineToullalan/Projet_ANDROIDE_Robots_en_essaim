from controller import Robot

def run_robot(robot):
    """Wall following robot"""
    
    
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6
    
    #enable motors
    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    #enable proximity sensors
    prox_sensors = []
    for ind in range(8):
        sensors_name = 'ps' + str(ind)
        prox_sensors.append(robot.getDistanceSensor(sensors_name))
        prox_sensors[ind].enable(timestep)
        
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        # Read the sensors:
        for ind in range(8):
            print("ind: {}, value: {}".format(ind, prox_sensors[ind].getValue()))
            
       
        # Process sensor data here.
        left_wall = prox_sensors[5].getValue() > 80
        left_corner = prox.sensors[6].getValue() > 80
        front_wall = prox_sensors[7].getValue() > 80
        
        left_speed = max_speed
        right_speed = max_speed
        
        if front_wall:
            print("Turn right in plane")
            left_speed = max_speed
            right_speed = -max_speed
        
        else:
        
            if left_wall:
                print("Drive froward")
                left_speed = max_speed
                right_speed = max_speed
            
            else:
                prinf("Turn left")
                left_speed = max_speed/8
                right_speed = max_speed
                
            if left_corner:
                print("Came too close, drive right")
                left_speed = max_speed
                right_speed = max_speed/8 
                    

        # Enter here functions to send actuator commands, like:
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
        

if __name__ == "__main__":

    # create the Robot instance.
    my_robot() = Robot()
    run_robot(my_robot)
    
