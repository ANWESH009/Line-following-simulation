"""tread-o-quest controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
motor_list=['front_left_motor','back_left_motor',
                'back_right_motor','front_right_motor']

sensor_list=['ir_ext_right','ir_ext_left','ir_right','ir_left','ir_middle']

sensor_list_wall=['front_sensor_right','front_sensor_left','right_sensor','left_sensor']


motor = dict()
sensor = dict()
wall_sensor= dict()

for m in motor_list:
    motor[m]=robot.getDevice(m)
    motor[m].setPosition(float('inf'))
    motor[m].setVelocity(0.0)
    
for s in sensor_list:
    sensor[s]=robot.getDevice(s)
    sensor[s].enable(timestep)

for s in sensor_list_wall:
    wall_sensor[s]=robot.getDevice(s)
    wall_sensor[s].enable(timestep)
  
def setMotorSpeed(r_speed,l_speed):
    motor['front_left_motor'].setVelocity(l_speed)
    motor['back_left_motor'].setVelocity(l_speed)
    motor['back_right_motor'].setVelocity(r_speed)
    motor['front_right_motor'].setVelocity(r_speed)
    
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
   
   
   
    right_value = sensor['ir_right'].getValue()
    ext_right_value = sensor['ir_ext_right'].getValue()
    left_value = sensor['ir_left'].getValue()
    ext_left_value = sensor['ir_ext_left'].getValue()
    middle = sensor['ir_middle'].getValue();
    
    print('ir_right\n',right_value)
    print('ir_ext_right\n',ext_right_value)
    print('ir_left\n',left_value)
    print('ir_ext_left\n',ext_left_value)
    print('ir_middle\n',middle)
    
    front_sensor_right_value = wall_sensor['front_sensor_right'].getValue()
    front_sensor_left_value = wall_sensor['front_sensor_left'].getValue()
    right_sensor_value = wall_sensor['right_sensor'].getValue()
    left_sensor_value = wall_sensor['left_sensor'].getValue()
    # Process sensor data here.
    print('front_sensor_right\n',front_sensor_right_value)
    print('front_sensor_left\n',front_sensor_left_value)
    print('right_sensor\n',right_sensor_value)
    print('left_sensor\n',left_sensor_value)
    
    if(right_value<900 and left_value>900):
        setMotorSpeed(5,0)
    elif(right_value>900 and left_value<900):
        setMotorSpeed(0,5)
    elif(ext_right_value>900 and right_value>900 and ext_left_value<900 ):
        setMotorSpeed(-5,5)
    #elif(ext_right_value<900 and right_value<900 and left_value<900 and ext_left_value<900 ):
        #setMotorSpeed(0,0)
    elif(ext_right_value<900 and right_value<900 and left_value<900 and ext_left_value<900):
        if(front_sensor_right_value<900 ):
            setMotorSpeed(5,-5)
        #elif(front_sensor_right_value>900 and front_sensor_left_value>900 and right_sensor_value>990 and left_sensor_value>900):
            #setMotorSpeed(0,0)
        else:
            if(right_sensor_value <900):
                setMotorSpeed(5,5)
        
            else:
                setMotorSpeed(0,1)
        
            if (front_sensor_right_value<900 and right_sensor_value <900 ):
                setMotorSpeed(3,0)   
        
    else:
        setMotorSpeed(5,5) 
        # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.

