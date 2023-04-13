import numpy as np
import yaml

from pid_controller import PIDController
from trajectory_controller import TrajectoryController
from cubic_spline_planner import CubicSplinePlanner
from trapezoidal_profile import TrapezoidalProfile

# Load configuration parameters from a YAML file
with open('config.yaml') as f:
    config = yaml.safe_load(f)

# Set up the PID controllers for velocity and trajectory control
velocity_pid = PIDController(config['velocity_pid']['kp'], config['velocity_pid']['ki'], config['velocity_pid']['kd'])
trajectory_pid = PIDController(config['trajectory_pid']['kp'], config['trajectory_pid']['ki'], config['trajectory_pid']['kd'])

# Load the waypoints from a file or define them in your program
waypoints = np.array([
    [0.0, 0.0],
    [1.0, 1.0],
    [2.0, 0.0],
    [3.0, 1.0],
    [4.0, 0.0],
])

# Generate a cubic spline trajectory based on the waypoints
spline = CubicSplinePlanner(waypoints)

# Compute the velocity profile for the trajectory
profile = TrapezoidalProfile(spline.s, config['max_speed'], config['max_acceleration'], config['sample_time'])

# Initialize the trajectory controller
trajectory_controller = TrajectoryController(trajectory_pid, spline.x, spline.y, spline.yaw, profile)

# Initialize the velocity controller
velocity_controller = PIDController(config['velocity_pid']['kp'], config['velocity_pid']['ki'], config['velocity_pid']['kd'])

# Main loop
while True:
    # Get the current position and orientation of the robot
    current_x = 0.0  # Replace with your code to get current position
    current_y = 0.0  # Replace with your code to get current position
    current_yaw = 0.0  # Replace with your code to get current orientation

    # Compute the linear and angular velocities based on the current position and the trajectory
    linear_velocity, angular_velocity = trajectory_controller.compute_control(current_x, current_y, current_yaw)

    # Compute the left and right wheel velocities based on the linear and angular velocities
    L = config['wheel_base']  # Replace with the distance between your robot's wheels
    vl = (2*linear_velocity - angular_velocity*L) / 2  # Compute left wheel velocity
    vr = (2*linear_velocity + angular_velocity*L) / 2  # Compute right wheel velocity

    # Update the velocity controller setpoint based on the desired linear velocity
    velocity_controller.set_setpoint(linear_velocity)

    # Compute the actual velocity of the robot and apply velocity control
    actual_velocity = 0.0  # Replace with your code to get actual velocity
    control_effort = velocity_controller.compute_control(actual_velocity, config['sample_time'])

    # Apply the control effort to the left and right wheel velocities using the velocity PID controller
    vl += control_effort
    vr += control_effort

    # Apply the left and right wheel velocities to the robot's motors
    # Replace with your code to apply the velocities to the motors
