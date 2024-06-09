#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import pygambit
from pygambit import gambit

class GamePublisher(Node):

    def __init__(self, player_x, player_y):
        super().__init__('game_publisher')

        self.twist_publisher = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.player_x = player_x
        self.player_y = player_y
        self.payoffs = {'player_1': [[-3, -3], [-5, 0]],
                        'player_2': [[0, -5], [-2, -2]]}
        self.expected_payoffs = []
        self.create_game = gambit.Game.from_dict(self.payoffs, 'Simple_Game')
        self.create_game.players[0].label = self.player_x                                 #  can add the label for players
        self.create_game.players[1].label = self.player_y
        
        print(f"Displaying Strategies available to the players: {([p.strategies for p in self.create_game.players ])}")
        print('\n')
        self.solve_game()

    def solve_game(self):

        mixed_staregy_nash = pygambit.nash.enummixed_solve(self.create_game)
        # print('Mixed startegy nash', mixed_staregy_nash)
        
        if mixed_staregy_nash:
            self.expected_payoffs = self.create_game.mixed_strategy_profile()                    # want to look for the expected utilities/payoffs for  the players 
            # print(self.expected_payoffs)
            return self.expected_payoffs
    
    def timer_callback(self):                                                             
        msg = Twist()
        msg.linear.x = self.expected_payoffs.strategy_value(self.create_game.players[1].strategies[1])
        '''can play the game repeatedly for more responses but for now just once'''
        self.twist_publisher.publish(msg=msg)


def main(args = None):
    rclpy.init(args=args)
    game_publisher = GamePublisher('Rakshit', 'Charles')
    try:
        
        rclpy.spin(game_publisher)
    except:
        game_publisher.destroy_node()
        rclpy.shutdown(game_publisher)
        

if __name__ == '__main__':
    main()