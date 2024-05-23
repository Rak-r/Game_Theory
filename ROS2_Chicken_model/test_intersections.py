import numpy as np


class RobotPath:
    def __init__(self, waypoints):
        self.waypoints = np.array(waypoints)

    def doesSegmentIntersectTrajectory(self, m, c):
        earliest_intersection = None
        closest_waypoint = None
        min_distance = float('inf')

        for i in range(len(self.waypoints) - 1):
            x0, y0 = self.waypoints[i]  # start point of the segment
            x1, y1 = self.waypoints[i + 1]  # end point of the segment

            # Calculate the parameter t for intersection
            denominator = (m * (x1 - x0) - (y1 - y0))
            if denominator != 0:
                t = (y0 - m * x0 - c) / denominator
                if 0.0 <= t <= 1.0:
                    # Find the intersection point
                    x = x0 + t * (x1 - x0)
                    y = y0 + t * (y1 - y0)
                    intersection = (x, y)

                    # Check if this is the earliest intersection
                    if earliest_intersection is None or t < earliest_intersection[0]:
                        earliest_intersection = (t, intersection)
                    
                    # Check if this is the closest waypoint
                    dist_start = (x - x0) ** 2 + (y - y0) ** 2
                    dist_end = (x - x1) ** 2 + (y - y1) ** 2
                    if dist_start < min_distance:
                        min_distance = dist_start
                        closest_waypoint = (x0, y0)
                    if dist_end < min_distance:
                        min_distance = dist_end
                        closest_waypoint = (x1, y1)

        if earliest_intersection:
            
            return earliest_intersection[1], closest_waypoint
        else:
            return None, None

# Example usage
waypoints = [(1, 1), (3, 4), (5, 2), (8, 2)]
m = 1.68
c = -1
robot_path = RobotPath(waypoints)
intersection_point, closest_waypoint = robot_path.doesSegmentIntersectTrajectory(m, c)
print("Intersection point:", intersection_point)
print("Closest waypoint:", closest_waypoint)