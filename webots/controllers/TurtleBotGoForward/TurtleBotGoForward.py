"""TurtleBotGoForward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor

TIME_STEP = 64
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()
keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)


leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))


while robot.step(TIME_STEP) != -1:
    key = keyboard.getKey()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    leftMotor = robot.getDevice("left wheel motor")
    rightMotor = robot.getDevice("right wheel motor")

    linear_speed = 0.0
    angular_speed = 0.0

    if key == ord('W') or key == ord('w'):
        linear_speed = MAX_SPEED
    elif key == ord('S') or key == ord('s'):
        linear_speed = -MAX_SPEED

    if key == ord('A') or key == ord('a'):
        angular_speed = MAX_SPEED
    elif key == ord('D') or key == ord('d'):
        angular_speed = -MAX_SPEED

    leftMotor.setVelocity(linear_speed - angular_speed)
    rightMotor.setVelocity(linear_speed + angular_speed)

    
