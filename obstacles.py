import pybullet as p
import time

# setting some constants for our robot's movements
turn_speed = 2      # speed when turning
forward_speed = 3   # speed when moving forward
obstacle_distance = 1.0  # distance threshold for detecting obstacles

# let's set up our simulation environment
def setup_simulation():
    print("setting up the simulation environment...")
    physics_client = p.connect(p.GUI)  # connect to the gui
    p.setGravity(0, 0, -9.8)  # adding gravity to our world
    plane_id = p.loadURDF("plane.urdf")  # load the ground
    robot_id = p.loadURDF("r2d2.urdf", [0, 0, 0.1])  # load our cute robot above the ground

    # placing some obstacles around for our robot to avoid
    print("adding obstacles...")
    obstacles = [
        p.loadURDF("cube.urdf", [1, 0, 0.5], globalScaling=0.5),
        p.loadURDF("cube.urdf", [2, 1, 0.5], globalScaling=0.5),
        p.loadURDF("cube.urdf", [3, -1, 0.5], globalScaling=0.5)
    ]
    
    return robot_id  # we return the robot id for later use

# function to control how fast our robot moves
def set_velocity(robot_id, left_speed, right_speed):
    print(f"setting velocities - left: {left_speed}, right: {right_speed}")
    p.setJointMotorControl2(robot_id, 0, p.VELOCITY_CONTROL, targetVelocity=left_speed)
    p.setJointMotorControl2(robot_id, 1, p.VELOCITY_CONTROL, targetVelocity=right_speed)

# function to check for obstacles in front of our robot
def get_distances(robot_id):
    print("checking for obstacles...")
    directions = {
        "left": (-0.5, 0),  # position for checking left
        "front": (1, 0),    # position for checking front
        "right": (0.5, 0)   # position for checking right
    }
    distances = {}  # we'll store distances here
    robot_position = p.getBasePositionAndOrientation(robot_id)[0]  # get robot's current position
    
    # loop through each direction to see if there's an obstacle
    for direction, (dx, dy) in directions.items():
        ray_end = [robot_position[0] + dx, robot_position[1] + dy, robot_position[2]]  # where the ray ends
        result = p.rayTest(robot_position, ray_end)[0]  # cast the ray
        distance = result[2] if result[0] != -1 else float('inf')  # get distance or 'inf' if no obstacle
        distances[direction] = distance  # store the distance for this direction
    
    return distances  # return all distances found

# deciding how our robot should move based on detected obstacles
def decide_and_move(robot_id, distances):
    print("deciding movement based on detected obstacles...")
    if distances["front"] < obstacle_distance:  # if something's in front
        print("obstacle detected in front!")
        if distances["left"] > distances["right"]:  # check which side is clearer
            turn_left(robot_id)  # turn left if it's clearer
        else:
            turn_right(robot_id)  # otherwise, turn right
    else:
        print("path is clear, moving forward!")
        move_forward(robot_id)  # no obstacles, so go forward

# helper functions to handle different robot movements
def move_forward(robot_id):
    set_velocity(robot_id, forward_speed, forward_speed)  # both wheels move forward

def turn_left(robot_id):
    set_velocity(robot_id, turn_speed, -turn_speed)  # left wheel goes forward, right wheel goes backward

def turn_right(robot_id):
    set_velocity(robot_id, -turn_speed, turn_speed)  # right wheel goes forward, left wheel goes backward

# main loop to run the simulation
def main():
    robot_id = setup_simulation()  # get our robot ready in the environment
    try:
        print("starting the simulation...")
        for step in range(1000):  # run the simulation for a set number of steps
            distances = get_distances(robot_id)  # check for obstacles
            decide_and_move(robot_id, distances)  # make a decision on movement
            p.stepSimulation()  # move the simulation forward
            time.sleep(0.01)  # pause a moment for smooth movement
    finally:
        print("disconnecting from simulation...")
        p.disconnect()  # make sure to disconnect cleanly

# let's run the main function if this script is executed
if __name__ == "__main__":
    main()
