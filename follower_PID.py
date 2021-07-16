"""follower_PID controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

ds= [];
ds_names = ["ir_right","ir_left","ir_ext_right","ir_ext_left","ir_front","ir_side_right","ir_side_left"];
ds_val = [0]*len(ds_names);

for name in ds_names:
    ds.append(robot.getDevice(name))
    ds[-1].enable(timestep)
    
wheels = []
wheel_names = ["front_right_wheel","front_left_wheel","rear_right_wheel","rear_left_wheel"]

for name in wheel_names:
    wheels.append(robot.getDevice(name))
    wheels[-1].setPosition(float('inf'))
    wheels[-1].setVelocity(0.0)
    
last_error = intg = diff = prop = waitCounter = 0
kp = 0.005
ki = 0
kd = 0.15

def pid(error):
    global last_error, intg , diff , prop, kp , ki, kd
    prop = error
    intg = error + intg
    diff = error - last_error
    balance = (kp*prop) + (ki*intg) + (kd*diff)
    last_error = error
    return balance
    
def setSpeed(base_speed, balance):
    wheels[0].setVelocity(base_speed + balance)
    wheels[1].setVelocity(base_speed - balance)
    wheels[2].setVelocity(base_speed + balance)
    wheels[3].setVelocity(base_speed - balance)
    
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    
    for i in range(len(ds)):
        ds_val[i] = ds[i].getValue()
        print(f"{ds_names[i]} : {ds_val[i]}" + "*"*40)
        
        
    if 1000 in ds_val[0:2]:
        
        if ds_val[0] > 900 and ds_val[1] > 900 and ds_val[2] < 900 and ds_val[3] < 900:
            setSpeed(5, 0)
            print("case 0")
        elif ds_val[0] > 900 and ds_val[1] > 900 and ds_val[2] > 900 and ds_val[3] < 900:
            setSpeed(2, -2)
            print("case 1")
        elif ds_val[0] > 900 and ds_val[1] > 900 and ds_val[2] < 900 and ds_val[3] > 900:
            setSpeed(2, 2)
            print("case 2")
        elif ds_val[0] < 900 and ds_val[1] > 900:
            setSpeed(5, 2)
            print("case 3")
        elif ds_val[0] > 900 and ds_val[1] < 900:
            setSpeed(5, -2)
            print("case 4")
        elif ds_val[0] > 900 and ds_val[1] > 900 and ds_val[2] > 900 and ds_val[3] > 900:
            print("case 5")
        else:
            setSpeed(0,0)
            print("case 6")
            
    else:
        if ds_val[4] != 1000:
            setSpeed(0, 3)
             
        else:
            error = (ds_val[5] - 700)
            rectify = pid(error)
            setSpeed(5, -rectify)
             
            
        
    
    pass

# Enter here exit cleanup code.
