#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from Chicken_speed_modulator_node import ChickenSpeedModulatorNode  # Replace 'your_module' with the actual module name



def simulate_pedestrian_trajectory(initial_x, initial_y, velocity_x, velocity_y, time_steps):
    trajectory = []
    for t in time_steps:
        x = initial_x + velocity_x * t
        y = initial_y + velocity_y * t
        trajectory.append((x, y))
    return np.array(trajectory)

def test_doesPlanIntersectTrajectory():
    rclpy.init()

    try:
        node = ChickenSpeedModulatorNode()

        # Simulate pedestrian path: moving diagonally across the x-y plane
        initial_ped_x = 0.0
        initial_ped_y = 0.0
        node.px_ms = 0.9  # Pedestrian speed in the x direction
        node.py_ms = 0.01  # Pedestrian speed in the y direction

        # Calculate slope and intercept
        node.slope = node.py_ms / node.px_ms if node.px_ms != 0 else float('inf')
        node.intercept = initial_ped_y - node.slope * initial_ped_x

        # Simulate robot path: moving along the y-axis at a constant speed
        node.waypoints = np.array([
            (1.0, -1.0),
            (1.0, 0.0),
            (1.0, 1.0),
            (1.0, 2.0),
            (1.0, 3.0),
            (1.0, 4.0),
            (1.0, 5.0)
        ])

        # Set the speeds
        node.robot_max_vel = 0.2  # Robot speed in the y direction
        node.ped_max_vel = np.sqrt(node.px_ms**2 + node.py_ms**2)  # Combined pedestrian speed

        # Time steps for simulation (every 0.5 seconds)
        time_steps = np.arange(0, 10, 0.5)  # Simulate for 10 seconds with a step of 0.5 seconds

        # Simulate pedestrian trajectory
        pedestrian_trajectory = simulate_pedestrian_trajectory(initial_ped_x, initial_ped_y, node.px_ms, node.py_ms, time_steps)
        print(pedestrian_trajectory)

        # Check for intersections at each time step
        intersection_times = []
        for t, (ped_x, ped_y) in zip(time_steps, pedestrian_trajectory):
            node.ped_pose_x = ped_x
            node.ped_pose_y = ped_y

            t_intersect = node.doesPlanIntersectTrajectory(node.waypoints, node.slope, node.intercept)
            if not np.isinf(t_intersect):
                intersection_times.append(t)

        # Print the results
        if intersection_times:
            print(f"Intersections detected at times: {intersection_times} seconds")
        else:
            print("No intersection detected.")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    test_doesPlanIntersectTrajectory()