"""line-follower controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
motor_list=['front_left_motor','back_left_motor',
                'back_right_motor','front_right_motor']

sensor_list=['ir_ext_right','ir_ext_left','ir_right','ir_left','ir_mid']
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
    print("left_speed = {}   right_speed = {}".format(l_speed,r_speed))
    motor['front_left_motor'].setVelocity(l_speed)
    motor['back_left_motor'].setVelocity(l_speed)
    motor['back_right_motor'].setVelocity(r_speed)
    motor['front_right_motor'].setVelocity(r_speed)
  
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
junction = 0 
prev_ext_right_value=1000
prev_right_value=1000
prev_mid_value=1000
prev_left_value=1000
prev_ext_left_value=1000
junction_flag=0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
       
    right_value = sensor['ir_right'].getValue()
    ext_right_value = sensor['ir_ext_right'].getValue()
    left_value = sensor['ir_left'].getValue()
    ext_left_value = sensor['ir_ext_left'].getValue()
    mid_value = sensor['ir_mid'].getValue()
    
    front_sensor_right_value = wall_sensor['front_sensor_right'].getValue()
    front_sensor_left_value = wall_sensor['front_sensor_left'].getValue()
    right_sensor_value = wall_sensor['right_sensor'].getValue()
    left_sensor_value = wall_sensor['left_sensor'].getValue()
    print('ir_right\n',right_value)
    print('ir_ext_right\n',ext_right_value)
    print('ir_left\n',left_value)
    print('ir_ext_left\n',ext_left_value)
    print('mid_sensor\n',mid_value)
    print('right_sensor\n',right_sensor_value)
    print('left_sensor\n',left_sensor_value)
    print("front_sensor_right_value\n",front_sensor_right_value)
    print("front_sensor_left_value\n",front_sensor_left_value)
    # Process sensor data here.
    if(((prev_ext_right_value-ext_right_value)>300) and ((prev_right_value-right_value)>300) and ((prev_left_value-left_value)>300) and ((prev_ext_left_value-ext_left_value)>300)):
        junction_flag+=1
        print(junction_flag)
        if(junction_flag==2):
            junction+=1
            junction_flag=0
            junction=math.floor((junction)/2)+1
            print("i found {} junction".format(junction))  
    #if(ext_right_value<700 and right_value<700 and  mid_value<700 and left_value<700 and ext_left_value<700):
        #junction+=1
   
        #print("i found {} junction".format(junction))
    if(ext_left_value >950 and ext_right_value > 950):
        print("line_follower")
        if( mid_value<700 and right_value>950 and left_value>950):
            print("line_follower_case_1")
            setMotorSpeed(5,5)
        elif(mid_value<700 and right_value<700 and left_value>950):
            print("line_follower_case_2")
            setMotorSpeed(0,3)
        elif(mid_value<700 and right_value>950 and left_value<700):
            print("line_follower_case_3")
            setMotorSpeed(3,0)
            
    elif((ext_left_value >950 and ext_right_value < 950)):
        print("edge_follower_1")
        if((mid_value<700 and right_value<700 and left_value>950)):
            print("edge_follower_1_case_1")
            setMotorSpeed(4,4) #go forward
        elif(mid_value>900 and right_value<700):
            print("edge_follower_1_case_2")
            setMotorSpeed(1,4) #right turn
        elif(left_value<700 and right_value<700):
            print("edge_follower_1_case_3")
            setMotorSpeed(4,1) #left turn
            
    elif((ext_left_value <950 and ext_right_value > 950)):
        print("edge_follower_2")
        if((mid_value<700 and left_value<700 and right_value>950)):
            print("edge_follower_2_case_1")
            setMotorSpeed(4,4) #go forward
        elif(mid_value>900 and left_value<700):
            print("edge_follower_2_case_2")
            setMotorSpeed(4,1) #left turn
        elif(right_value<700 and left_value<700):
            print("edge_follower_2_case_3")
            setMotorSpeed(1,4) # right turn
    if(mid_value>900 and (right_sensor_value<900 or left_sensor_value<900)):
        
        print("wall_follower")
        #right_value>900 and  ext_right_value>900 and left_value>900 and ext_left_value >900 and  
        if((right_sensor_value<900 or left_sensor_value<900) and (front_sensor_right_value>900 and front_sensor_left_value>900)):
            print("wall_follower_case_1")
            if(left_sensor_value<900):
                if(left_sensor_value>280):
                    print("wall_follower_case_1_L_1")
                    setMotorSpeed(2,1)
                else:
                    print("wall_follower_case_1_L_2")
                    setMotorSpeed(2,2)
            elif(right_sensor_value<900):
                if(right_sensor_value>280):
                    print("wall_follower_case_1_R_1")
                    setMotorSpeed(1,2)
                else:
                    print("wall_follower_case_1_R_2")
                    setMotorSpeed(2,2)
            #setMotorSpeed(2,2)
        elif(right_sensor_value<900 and (front_sensor_right_value<900 or front_sensor_left_value<900) and left_sensor_value>900):
             
            print("wall_follower_case_2")
            setMotorSpeed(2,0)    
        elif(right_sensor_value>900 and (front_sensor_right_value<900 or front_sensor_left_value<900) and left_sensor_value<900):
             
            print("wall_follower_case_3")
            setMotorSpeed(0,2)      
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
