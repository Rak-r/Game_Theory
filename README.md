This repository explores the operations with Game Theory for decision making. 
I have been researching about different ways to integrate sequential games for decision making into robotics.

# Integrating ROS2 and Gambit

1. The script `Test_Game_Theory_ROS2.py` includes a simple game similar to `Prisonor's Dilemma` created using gambit.
2. To test the gambit connection with ROS2 Twist commands, a simple payoff outputs are assigned only to `linear.x` field.
3. To test the working with `Gazebo  Garden / ignition Fortress`, we can create a `ros_gz_bridge` with the following command.
   * `ign gazebo ackermann_steering.sdf` or `gz sim ackermann_steering.sdf`
   * `ros2 run ros_gz_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist`.
   * In the new terminal : `python3 Test_Game_theory_ROS2.py`

4. In the terminal, check `ros2 topic list` to check the published topic.
5. We can visualise the robot moving with the output of the game's payoff.

##### Note: This script shows an initial insights to integrate game outputs with ROS2 and testing in simulation. In future more in-depth exploration will be provided.
