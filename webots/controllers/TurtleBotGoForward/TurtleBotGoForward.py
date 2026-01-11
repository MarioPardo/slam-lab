"""TurtleBotGoForward controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor
from robot_comm import RobotPublisher, RobotSubscriber

TIME_STEP = 64
MAX_SPEED = 6.28

# create the Robot instance.
robot = Robot()
keyboard = robot.getKeyboard()
keyboard.enable(TIME_STEP)

#Drive Setup
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))


#Lidar Setup
lidar = robot.getLidar("lidar")
lidar.enable(TIME_STEP)

# ZMQ Publisher Setup (send data to C++)
publisher = RobotPublisher()
# ZMQ Subscriber Setup (receive responses from C++)
subscriber = RobotSubscriber()
subscriber.subscribe("response")  # Subscribe to response topic

message_counter = 0
fake_data_value = 42  # Hardcoded test data


while robot.step(TIME_STEP) != -1:
    
    key = keyboard.getKey()
    timestep = int(robot.getBasicTimeStep())

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

    #Lidar

    range_image = lidar.getRangeImage()
    lidar.enablePointCloud()
    
    # Send fake data every 100 timesteps
    message_counter += 1
    if message_counter % 100 == 0:
        publisher.publish_data(fake_data_value)
        print(f"[Python] Sent data: {fake_data_value}")
        fake_data_value += 1  # Increment for next time
    
    # Check for responses from C++ (non-blocking)
    response = subscriber.receive_message(timeout_ms=0)
    if response:
        topic, msg = response
        print(f"[Python] Received from C++ on '{topic}': {msg}")
    



    
