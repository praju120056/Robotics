import pybullet as p
import time
import pybullet_data
import numpy as np
import random

# Start the PyBullet simulation with GUI
p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity and load the ground plane
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

# Load the Husky robot model
robot_id = p.loadURDF("husky/husky.urdf", basePosition=[0, 0, 0], useFixedBase=False)

# Husky's wheel joint indices
wheel_joints = [2, 3, 4, 5]  # These are the four wheels

# Load cubes: one in the way and two at random locations
cube_height = 0.1
cubes = []

# Function to calculate distance between two positions
def calculate_distance(pos1, pos2):
    return np.linalg.norm(np.array(pos1) - np.array(pos2))

# Fixed position of the cube in the way
blocking_cube_pos = [6, 0, cube_height]  # Directly in the path of the rover
cubes.append(p.loadURDF("cube.urdf", basePosition=blocking_cube_pos, useFixedBase=False))

# Generate two additional cubes at random positions, ensuring they are at least 6 units away
for _ in range(2):
    while True:
        # Generate random positions
        x_pos = random.uniform(10, 16)  # Ensure cubes are beyond 6 units from the origin
        y_pos = random.uniform(-2, 2)    # Random y position between -2 and 2
        if calculate_distance([0, 0, 0], [x_pos, y_pos, cube_height]) > 6:  # Check distance
            cubes.append(p.loadURDF("cube.urdf", basePosition=[x_pos, y_pos, cube_height], useFixedBase=False))
            break

# Set the mass and friction properties of the cubes
for cube_id in cubes:
    p.changeDynamics(cube_id, -1, mass=10.0)  # Set mass to a larger value
    p.changeDynamics(cube_id, -1, lateralFriction=1.0, spinningFriction=1.0, rollingFriction=1.0)

# Define detection parameters and control constants
detection_distance = 2.0  # Set detection distance to 2
FORWARD_VELOCITY = 3.0     # Set speed of the robot to normal (3.0)
BACKWARD_VELOCITY = -3.0
TURN_VELOCITY = 8.0        # Set turning velocity to 8.0 for faster turns
TURN_TIME = 0.1            # Set turn time to 0.1 seconds for quicker turns

# Avoid an obstacle
def avoid_obstacle(robot_id):
    # Move backward
    for joint in wheel_joints:
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL, targetVelocity=BACKWARD_VELOCITY)
    time.sleep(0.5)  # Back up for half a second

    # Turn right (rotate wheels in opposite directions)
    for joint in wheel_joints[:2]:  # Front wheels
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL, targetVelocity=TURN_VELOCITY)
    for joint in wheel_joints[2:]:  # Back wheels
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL, targetVelocity=-TURN_VELOCITY)
    time.sleep(TURN_TIME)

    # Move forward after turning
    for joint in wheel_joints:
        p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL, targetVelocity=FORWARD_VELOCITY)

# Set the total simulation time (in seconds)
total_simulation_time = 30  # Run for 30 seconds
start_time = time.time()

# Main simulation loop
try:
    while True:
        current_time = time.time()
        elapsed_time = current_time - start_time

        if elapsed_time > total_simulation_time:
            print("Simulation time completed.")
            break  # Exit the loop after 30 seconds

        p.stepSimulation()
        robot_pos, _ = p.getBasePositionAndOrientation(robot_id)

        # Check for detected obstacles
        obstacle_detected = False
        for cube_id in cubes:
            cube_pos, _ = p.getBasePositionAndOrientation(cube_id)
            distance = calculate_distance(robot_pos, cube_pos)

            # If the robot is within detection distance, avoid the obstacle
            if distance < detection_distance:
                print("Obstacle detected! Avoiding obstacle.")
                avoid_obstacle(robot_id)
                obstacle_detected = True
                break  # Avoid multiple detections in one loop

        # If no obstacles are detected, continue moving forward
        if not obstacle_detected:
            for joint in wheel_joints:
                p.setJointMotorControl2(robot_id, joint, p.VELOCITY_CONTROL, targetVelocity=FORWARD_VELOCITY)

        time.sleep(1 / 240)  # Adjusted render time

except KeyboardInterrupt:
    print("Simulation ended by user.")
    p.disconnect()

except Exception as e:
    print(f"An error occurred: {e}")
    p.disconnect()
