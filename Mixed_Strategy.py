'''This script creates a simple game called 'Matching Pennies, in which both players simulataneously reveals a penny/coin.
If both coins showed heads or both showed tails, Player 1 gets $1 from Player 2. 
But if one of the pennies showed tails or heads, then Player 2 gets $1
from Player 1.'''


'''There are two action/strategies available--> Heads and Tails
Payoffs for each players: 
  
     __           __
P1 =|               |
    |  1       -1   | --> Heads
    |  -1        1  | --> Tails
    |__           __| ,

     __ Heads   Tails __
P2 =|                   |
    |  -1       1       |
    |   1       -1      |
    |__               __|    

    
I have calculated the mixed srategy nash equilibrium manually to verify the concept and to understand the working of Gambit API and methods
 
'''



import pygambit 
from pygambit import gambit
import numpy as np

class MatchingPennies():

    def __init__(self, player_1, player_2):

        self.payoffs = {'player_1': [[1, -1], [-1, 1]],
                        'player_2': [[-1, 1], [1, -1]]}

        self.player_1 = player_1
        self.player_2 = player_2

        self.create_game = gambit.Game.from_dict(self.payoffs,"Matching pennies")         #  gambit method to create a game
        self.create_game.players[0].label = self.player_1                                 #  can add the label for players
        self.create_game.players[1].label = self.player_2
        
        print(f"Displaying Strategies available to the players: {([p.strategies for p in self.create_game.players ])}")
        print('\n')

        self.solve_game()

    def solve_game(self):
        '''As you can observe, the payoffs provided above represents a zero-sum game meaning cancelling each other, 
        so there will be no pure nash equilibirum exists in such cases'''

        pure_nash_equilibrium = pygambit.nash.enumpure_solve(self.create_game)
        print('Pure Nash equilibrium', pure_nash_equilibrium)

        '''However, there must exist a mixed strategy nash equilibrium which can be computed using enummixed method'''
        mixed_staregy_nash = pygambit.nash.enummixed_solve(self.create_game)
        print('Mixed startegy nash', mixed_staregy_nash)
        
        if mixed_staregy_nash:
            
            expected_payoffs = self.create_game.mixed_strategy_profile()                    # want to look for the expected utilities/payoffs for  the players 
            print('expected_payoffs:', expected_payoffs)


def main():
    matching_pennies = MatchingPennies('Rakshit', 'Charles')
    print('game is created', matching_pennies.create_game)

if __name__ == '__main__':
    main()
