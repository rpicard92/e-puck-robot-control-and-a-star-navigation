"""astar_python_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, LED, DistanceSensor
from controller import *
#from controller import DifferentialWheels


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
  #led = robot.getLED(0)
 # ds = robot.getDistanceSensor(0)
  #ds.enable(timestep)


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #val = ds.getValue()

    #differentialWheels = DifferentialWheels(robot)
    #differentialWheels.setSpeed(100,100)
    # Process sensor data here.
    # Enter here functions to send actuator commands, like:
    #led.set(1)
    #pass
    log("Test")
    robot.step();
    
# Enter here exit cleanup code.
