import pygambit
from pygambit import gambit
import numpy as np
class PrisonersDilemma():

    def __init__(self, player_1, player_2):
        
        self.player_1 = str(player_1)                                                    # assign player 1 attribute
        self.player_2 = str(player_2)                                                    # assign  player 2 attribute
        
        '''Can create the game using new_table, from_arrays method, preferred method is from_dict because of structured key-value payoff pairs for each player'''
        self.payoffs = {'P1_payoff':np.array([[-2,-5],[0,-3]]),                 
                   'P2_payoff': np.array([[-3,0],[-5,-3]])}
        

        self.create_game = gambit.Game.from_dict(self.payoffs,"A Prisonor's Dilemma")      # gambit method to create a game
        self.create_game.players[0].label = self.player_1                                 #  can add the label for players
        self.create_game.players[1].label = self.player_2
        

        print(f"Displaying Strategies available to the players: {([p.strategies for p in self.create_game.players ])}")
        print('\n')

        self.solvegame()                                                                  # call the function to solve the game


    def solvegame(self):
        # payoffs = {'P1_payoff':np.array([[-2,-5],[0,-3]]),
        #            'P2_payoff': np.array([[-2,0],[-5,-3]])}
        nash_equilibria = pygambit.nash.enumpure_solve(self.create_game)                  # used enumpure_solver to compute pure startegy nash equilibria

        return print('Pure startegy Nash Equilibrium using enumpure method:', nash_equilibria)

def main():

    obj = PrisonersDilemma('Rak', 'Charles')
    print('Game is done creating:',obj.create_game)
    print('\n','Game is solved')

if __name__ == '__main__':
    main()
