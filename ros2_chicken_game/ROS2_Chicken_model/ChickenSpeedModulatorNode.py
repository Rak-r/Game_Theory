#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray, Twist, PoseWithCovarianceStamped, Point
import tf2_ros
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray
import message_filters
from nav_msgs.msg import Path
import numpy as np
from nav2_simple_commander.line_iterator import LineIterator
from sequentialChicken import ChickenGame
import random
from time import time
from visualization_msgs.msg import Marker
from rclpy.duration import Duration
from pod2_msgs.msg import GameInfo
import pdb

class ChickenSpeedModulatorNode(Node):

    def __init__(self):
        super().__init__('chicken_speed_modulator_node')

        self.get_logger().info('Chicken Modulator node is initialised')
        # subscribers
        
        self.robot_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/localization_pose', self.robot_pose_cb, 10)       # the robot pose in global frame (map)
        self.robot_vel_callback = self.create_subscription(Odometry, '/rtabmap_odom', self.robot_vel_cb, 10)
        self.global_plan_sub = self.create_subscription(Path, '/plan', self.robot_global_plan_cb, 10)                                 # path message generated  y nav2 global planner
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)                                           # orginal twist generated by nav2 
        

        # add message filter synchronizer for pedestrian related messages to avoid time inaccuracies
        self.pedestrian_pose_sub = message_filters.Subscriber(self, DetectionArray, '/yolo/detections_3d')                            # pedestrian pose in global frame (map)
        self.pedestrian_vel_sub = message_filters.Subscriber(self, DetectionArray, '/yolo/detections_speed')                          # pedestrian speed topic
        self.synchronizer = message_filters.ApproximateTimeSynchronizer((self.pedestrian_pose_sub, self.pedestrian_vel_sub), 10, 0.5)
        self.synchronizer.registerCallback(self.Pedestrian_state_cb)
        
        # publisher
        self.modulated_twist_pub = self.create_publisher(Twist, '/cmd_vel_chicken_modulated', 10)                       # cmd_vel modulated publisher after game's decision
        self.marker_pub = self.create_publisher(Marker, '/collsion_point', 10)
        self.game_info_pub = self.create_publisher(GameInfo, '/chicken_game_info', 10)                                  # publisher for game output data
        self.monitor_callbacks = self.create_timer(1.0, self.timer_callback)                                     # timer callback to check messages recived or not

        # ROBOT SPECIFIC VARIABLES

        self.robot_pose = Pose()
        self.robot_pose_x = 0.0                                                                                         # initialize robot pose
        self.robot_pose_y = 0.0
        self.robot_max_vel = 0.2                                                                                        # NOTE set maximum robot speed (assuming FAST)
        self.robot_vel = 0.0                                                                                            # magnitude of current velocity
        
        self.waypoints = []                                                                                             # list to store extracted waypoint information from NAV2 Path message
        self.is_plan = False
        
        # PEDESTRIAN SPECIFIC VARIABLES
        self.ped_pose = Pose()
        self.px_ms = np.nan                                                                                              
        self.py_ms = np.nan
        self.ped_max_vel = 0.2                                                                                          # Assume ped and robot travels at same speed because we are only intrested in the collsion cases.
        self.ped_pose_x = np.nan                                                                                        # nan means there is no pedestrian in the scene
        self.ped_pose_y = np.nan 
        self.is_pedestrian = False                                                                                      # bool to catch if there is pedestrian or not

        # create instance of chicken game class
        self.game_of_chicken = ChickenGame()
        self.V,self.S = self.game_of_chicken.solveGame(U_crash_Y=-100, U_crash_X=-100, U_time=1., NY=30, NX=30)         # solve the chicken game
        self.b_yield = False
        self.old_time = time()

    '''
    callback to subscribe to NAV2 Path message from global planner server
    '''
    def robot_global_plan_cb(self, msg):
       
        waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]                                 # extract the waypoints of the global plan
        self.waypoints = np.array(waypoints)
        # print(self.waypoints)
            
    '''
    callback to subscribe to RTABMAP SLAM'S Pose message for robot pose info in MAP frame
    ''' 
    def robot_pose_cb(self, msg):
        
        pose = PoseWithCovarianceStamped()
        
        self.robot_pose = msg.pose.pose
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        
    '''
    callback to subscribe to Yolo output to get pedestrian position in world frame
    '''
    def Pedestrian_state_cb(self, pose_msg: DetectionArray, speed_msg: DetectionArray):
        
        
        # self.is_pedestrian = True
        # # TODO, this needs to go somewhere else because this callback will get triggered only when there are incoming messages
        # if not speed_msg:
        #     self.px_ms = np.nan
        #     self.py_ms = np.nan
        #     self.ped_pose_x = np.nan
        #     self.ped_pose_y = np.nan
        
        for detection in speed_msg.detections:
            self.is_pedestrian = True
            # self.px_ms = detection.velocity.linear.x
            # self.py_ms = detection.velocity.linear.y
            self.px_ms = 0.0                                   # HACK  Assumming pedestrian is moving in Y in ROS cordinate space
            self.py_ms = 1.0
           
        for detection in pose_msg.detections:
            
            self.is_pedestrian = True
            # store the x and y positions of the detected and tracked pedestrian
            
            self.ped_pose_x = detection.bbox3d.center.position.x
            self.ped_pose_y = detection.bbox3d.center.position.y
            self.ped_pose = detection.bbox3d.center
    '''
    callback to subscribe to NAV2 generated twist message
    '''  
    def robot_vel_cb(self, msg):
        self.robot_vel = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
    
    '''
    Function to keep track if the callback is triggered or not and reset the attributes.
    '''
    def timer_callback(self):

        if self.is_plan:
            self.waypoints = np.nan
        if not self.is_pedestrian:
            self.px_ms = np.nan
            self.py_ms = np.nan
            self.ped_pose_x = np.nan
            self.ped_pose_y = np.nan
        
        self.is_pedestrian = False
        self.is_plan = False

    def publish_collision_marker(self, x, y, r,g,b, scale):                                                         # Marker message to visualize the spatial and temporal intersection/collision point.
        marker = Marker()
        marker.header.frame_id = "map"
        # marker.header.stamp = self.get_clock().now().nanoseconds / 1e9
        marker.ns = "collision_points"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.lifetime = Duration(seconds=0.5).to_msg()

        self.marker_pub.publish(marker)

    '''
    1.) Function to calculate the distance between a point and the line

    2.) returns: condition to check, distance between the single NAV2 pose/waypiont or pedestrain slope-intercept
    
    '''
    #https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Another_formula
    def doesWaypointIntersectPedPath(self,wx,wy, m, c):  
        min_dist = 0.2
        a = m
        b = -1
       
        numerator = abs(a * wx + b * wy + c)
        denominator = np.sqrt(a**2 + b**2) 
        distance = numerator / denominator
        # self.get_logger().info(f'spatial: {distance}')
        return (distance < min_dist)

    '''
    Need to find the closest point on ped line from robot current pose
    '''
    def closest_point_on_line(self, wx, wy, m, c):
        # Line equation: y = mx + c
        # Perpendicular line from (rx, ry): y = (-1/m)x + b
        # Find b for the perpendicular line: ry = (-1/m)rx + b => b = ry + (rx / m)
        
        b_perp = wy + (wx / m)
        
        # Intersection point of the two lines: mx + c = (-1/m)x + b_perp
        # Solve for x: mx + (1/m)x = b_perp - c => x(m + 1/m) = b_perp - c => x = (b_perp - c) / (m + 1/m)
        
        x_closest = (b_perp - c) / (m + 1/m)
        y_closest = m * x_closest + c
        
        return x_closest, y_closest
   
    '''
    REAL WORK HERE  !!!  Function to check the test if a single pose and speed will collide close to a temporal trajectory
    '''
    def doesWaypointIntersectTrajectory(self, wx,wy,m,c, speedped, speedrobot, current_robot_x, current_robot_y):  
    
         # Handle zero division for speeds
        if speedrobot < 0.001:                                                                                       # no need for abs becuase taking magnitudes
            speedrobot = 0.01                                                                                        # Set minimum speed for robot
        if speedped < 0.001:
            speedped = 0.01                                                                                          # Set minimum speed for pedestrian

        if not self.doesWaypointIntersectPedPath(wx, wy, m, c):                                                          # if path dont intersect , no way trajectory will intersect
            return (np.inf, np.inf, np.inf, np.inf)                                                                                        # inf means there is no intersection
       
        # x_closest, y_closest = self.closest_point_on_line(wx, wy, m, c)
        # self.publish_collision_marker(x_closest, y_closest, 1.,0.,0., 0.2)
        x_closest = wx
        y_closest = wy
        distance_robot_to_closest = np.sqrt((current_robot_x - x_closest)**2 + (current_robot_y - y_closest)**2)                               # TODO use the curve path instead of assuming straight
        distance_ped_to_closest = np.sqrt((self.ped_pose_x - x_closest)**2 + (self.ped_pose_y - y_closest)**2)
        
        time_robot_to_collsion = distance_robot_to_closest/speedrobot
        time_ped_to_collision = distance_ped_to_closest/speedped

        # self.get_logger().info(f'rd: {round(distance_robot_to_closest,3)}, r_cur: {round(current_robot_x,3)},pd: {round(distance_ped_to_closest,3)},  x_c: {x_closest}, y_c: {y_closest}, r_cur: {round(current_robot_x,3), round(current_robot_y,3)}, p_cur: {round(self.ped_pose_x,3), round(self.ped_pose_y,3)}')
        
        if abs(time_robot_to_collsion-time_ped_to_collision) <= 0.5:   #check for temporal collision
            # self.publish_collision_marker(x_closest, y_closest, 0.,0.,1.,0.3 )
            # pdb.set_trace()
            return (time_robot_to_collsion, time_ped_to_collision, distance_robot_to_closest, distance_ped_to_closest)
        else:
            return (np.inf, np.inf, np.inf, np.inf) 


    def cmd_vel_cb(self, msg):
        game_info_msg = GameInfo()
        curr_time = time()
        delta_time = curr_time - self.old_time
        b_reversing = False
        if abs(self.px_ms) <= 0.0001:                                                                                      # handle zero-division
            self.px_ms = 0.001
        if abs(self.py_ms) <=0.0001:
            self.py_ms = 0.001
        if not self.is_pedestrian:   
            # msg.angular.z = 0.0  
            # msg.linear.x = 0.2                                                                              # if no pedestrian , do nothing                
            self.modulated_twist_pub.publish(msg)
            # print('hello1')
            return
        # print('detected')
        if msg.linear.x < 0.0:
                b_reversing = True
                self.modulated_twist_pub.publish(msg)
                return
       
        #ped_speed = np.sqrt((self.px_ms)**2 + (self.py_ms)**2)
        m = self.py_ms/self.px_ms     #gradient pf ped trajectory in map frame
        c = self.ped_pose_y - m*self.ped_pose_x                                                                       # c = y - mx
       
        b_spatialCollision = True   #have we found a temporal collision?                                          
       
        # for waypoint in self.waypoints:
            
        #     wx, wy = waypoint 
                           
        #     # (time_robot_to_collsion, time_ped_to_collision, distance_robot_to_closest, distance_ped_to_closest) = self.doesWaypointIntersectPedPath(wx,wy, m,c)
        #     if self.doesWaypointIntersectPedPath(wx,wy, m,c):
        #         b_spatialCollision = True
                
        #         break
        wx =  4.155768871307373 #4.105230331420898


        wy = -2.798013925552368 #-2.8832411766052246

        
        # if not self.is_pedestrian:
        #     b_spatialCollision = True
        #     return
        b_update = False
        if b_spatialCollision:
            
            b_update = True
            distance_robot_to_closest = np.sqrt((self.robot_pose_x - wx)**2 + (self.robot_pose_y - wy)**2)                               # TODO use the curve path instead of assuming straight
            distance_ped_to_closest = np.sqrt((self.ped_pose_x - wx)**2 + (self.ped_pose_y -wy)**2)

            time_robot_to_closest = distance_ped_to_closest/0.2  # robot speed
            time_ped_to_closest = distance_ped_to_closest/0.4   # assume ped moves slow  

            
            # X = int(np.floor(time_robot_to_closest))                                                               # Discretise the points to nearest ints                                                                                                             # NOTE Quantise points to 1 meter boxes
            # Y = int(np.floor(time_ped_to_closest))

            X = int(np.ceil(distance_robot_to_closest*2.5))                                                               # Discretise the points to nearest ints                                                                                                             # NOTE Quantise points to 1 meter boxes
            Y = int(np.ceil(distance_ped_to_closest*1.0))


            # if we played chicken once, then do the original but if the time played chicken exceeds some thresh again play
            # b_update=False
            # if delta_time > 0.5:                                                                                      # NOTE check time is in seconds not millis
            #     b_update=True
                #lets play chicken
            prob_robot_yield = self.S[Y, X, 0]  
                                                                                                                    # Action the robot should take either yeild (SLOW) or not yield, carry on (FAST)                        
            prob_ped_yield = self.S[Y, X, 1]   
            r1 = random.random()                                                                                  # Create a random probabilty range (prob. is max 1)
            if r1 < prob_robot_yield:                                
                self.b_yield =  True                                                                             # Slow down the robot by halving the speed in linear.x (fwd/ bkd velocity)  
            else:
                self.b_yield = False     

            if X==Y:

                # generate the ROS2 game info message to fill in the information,  we only capture the data when we are at diagonal
                
                self.publish_collision_marker(wx, wy, 1.,0.,1.,0.3 )

            else:    
                self.publish_collision_marker(wx, wy, 0.,0.,1.,0.3 )
           
            #debug info
            np.set_printoptions(threshold=np.inf, precision=3, linewidth=200, suppress=True)
            # self.get_logger().info(f'X: {X}, Y: {Y}, y:{self.b_yield}, PRY: {prob_robot_yield}, PPY: {prob_ped_yield}, upte: {b_update}')
            self.old_time = curr_time
            if self.b_yield:
                msg.linear.x/=2
            
            

            game_info_msg.header.stamp = self.get_clock().now().to_msg()
            game_info_msg.header.frame_id = 'map'
            game_info_msg.game_played = b_update
            game_info_msg.yielding = self.b_yield
            game_info_msg.player1discrete = X
            game_info_msg.player2discrete = Y
            game_info_msg.player1yieldprob = prob_robot_yield
            game_info_msg.player2yieldprob = prob_ped_yield
            game_info_msg.player1_pose = self.robot_pose
            game_info_msg.player2_pose = self.ped_pose
            
            self.game_info_pub.publish(game_info_msg)   
        # else:
        #     game_info_msg.header.stamp = self.get_clock().now().to_msg()
        #     game_info_msg.game_played = False
        #     game_info_msg.yielding = self.b_yield
        #     game_info_msg.player1discrete = np.nan
        #     game_info_msg.player2discrete = np.nan
        #     game_info_msg.player1yieldprob = np.nan
        #     game_info_msg.player2yieldprob = np.nan

            self.get_logger().info(f'X: {X}, Y: {Y}, rd: {round(distance_robot_to_closest,3)}, r_cur: {round(self.robot_pose_x,3), round(self.robot_pose_y,3)},pd: {round(distance_ped_to_closest,3)},  x_c: {wx}, y_c: {wy}, p_cur: {round(self.ped_pose_x,3), round(self.ped_pose_y,3)}, m: {m}, c: {c}, PRY: {prob_robot_yield}, PPY: {prob_ped_yield}, upte: {b_update}')
                                                                           # publish only when in the game
        # msg.linear.x =0.0
        self.modulated_twist_pub.publish(msg)
       
def main():
    rclpy.init()
    node = ChickenSpeedModulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




    





