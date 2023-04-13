import numpy as np

class TrajectoryPlanner:
    def __init__(self, waypoints, max_velocity, max_acceleration):
        self.waypoints = waypoints
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.linear_velocities = None
        self.angular_velocities = None
        self.total_distance = None
        self.time_steps = None
        
    def generate_trajectory(self, time_step=0.1):
        self.linear_velocities, self.angular_velocities = self.calculate_velocities(self.waypoints, self.max_velocity, self.max_acceleration)
        self.total_distance = np.sum(self.calculate_distance(self.waypoints))
        self.time_steps = np.arange(0, self.total_distance, time_step)
        linear_distances = np.zeros_like(self.time_steps)
        angular_distances = np.zeros_like(self.time_steps)
        for i in range(1, len(self.time_steps)):
            dt = self.time_steps[i] - self.time_steps[i-1]
            linear_distances[i] = linear_distances[i-1] + self.linear_velocities[i-1] * dt
            angular_distances[i] = angular_distances[i-1] + self.angular_velocities[i-1] * dt
        return linear_distances, angular_distances
    
    def calculate_velocities(self, waypoints, max_velocity, max_acceleration):
        distances = self.calculate_distance(waypoints)
        total_distance = np.sum(distances)
        times = distances / max_velocity
        for i in range(1, len(times)):
            times[i] = max(times[i], times[i-1] + distances[i-1] / max_velocity)
        linear_velocities = max_velocity * np.ones_like(times)
        for i in range(1, len(times)):
            linear_velocities[i] = min(linear_velocities[i], (distances[i] - distances[i-1]) / (times[i] - times[i-1]))
            linear_velocities[i-1] = min(linear_velocities[i-1], (distances[i] - distances[i-1]) / (times[i] - times[i-1]))
        for i in range(1, len(linear_velocities)):
            linear_velocities[i] = min(linear_velocities[i], linear_velocities[i-1] + max_acceleration * (times[i] - times[i-1]))
        angular_velocities = np.zeros_like(linear_velocities)
        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i-1][0]
            dy = waypoints[i][1] - waypoints[i-1][1]
            theta = np.arctan2(dy, dx)
            angular_velocities[i-1] = (theta - np.arctan2(waypoints[i-1][1], waypoints[i-1][0])) / times[i-1]
        return linear_velocities, angular_velocities
    
    def calculate_distance(self, waypoints):
        distances = []
        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i-1][0]
            dy = waypoints[i][1] - waypoints[i-1][1]
            distances.append(np.sqrt(dx*dx + dy*dy))
        return np.array(distances)
