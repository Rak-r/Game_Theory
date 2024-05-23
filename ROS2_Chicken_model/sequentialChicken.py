#!/usr/bin/env python3

#at each turn, players can go FAST=speed 2 units/tick , or SLOW=1 unit/tick
#in the absense of collisions, they would want to go FAST, reaching the destination in distance/2 ticks
#hence the divisions of utilities by two throughout.

#we use Gauge Invariance to always assume that t=0 at the time of the crash.
#(this allows us to work with U(y,x) rather than U(y,x,t) )
#it is a bit of a hack.
#U_time is the positive value of 1 second (saved) in dollars.
#U_crash is negtive value of crashing.
#/2 occurs where we assume a player is moving at speed 2 and convert time=distance/speed.
#20180202: change so that python indices really are y and x
#use fanta's scheme so crashes are at 0,0 and 1,1 only.
#and assign (0,.5*U_d*x/2) , (.5*U_d*y/2, 0) to outer 2 rows and 2 cols. (using gauge invar)

#20200706 IDEA finding big nashes is computationally complex, can we use GPU/TF to accellerate?

import numpy as np
from pylab import *
import pygambit
import nashpy
import pdb
import random
import solutionConcepts


class ChickenGame():


    def __init__(self):
         
         pass
    def valueMatrixEndCases(self,U_crash_Y=-100, U_crash_X=-100, U_time=1., NY=20, NX=20):   #2020-07-02 tweaking edge cases
        V = np.zeros((NY,NX,2))   #vals as pairs in 3rd dim

        #successful passing end states  [y,x,player_to_reward Y=0]
        Y=0 ; X=1
        for x in range(1,NX):    #Y has won
            V[0,x,Y] = 0.           
            V[0,x,X] = -U_time*(x/2.)  
        for x in range(2,NX):
            V[1,x,Y] = -0.5   #UPDATED            
            V[1,x,X] = -U_time*((x)/2.)  #UPDATED
        for y in range(1,NY):    #X has won
            V[y,0,X] = 0. 
            V[y,0,Y] = -U_time*(y/2.)  
        for y in range(2,NY):
            V[y,1,X] = -0.5  #UPDATED           
            V[y,1,Y] = -U_time*((y)/2.)   #UPDATED
        #crash end states -- overwrite
        V[0,0,Y] = U_crash_Y
        V[0,0,X] = U_crash_X
        V[1,1,Y] = U_crash_Y
        V[1,1,X] = U_crash_X
        return V



    ##### OLD APPROACH ##########
    #############################
    
    def valueMatrixEndCasesOld(self, U_crash_Y=-100, U_crash_X=-100, U_time=1., NY=20, NX=20):   #as used up to 2020-07-02
        V = np.zeros((NY,NX,2))   #vals as pairs in 3rd dim
        #successful passing end states  [y,x,player_to_reward Y=0]
        Y=0 ; X=1
        for x in range(1,NX):    #Y has won
            V[0,x,Y] = 0.           
            V[0,x,X] = -U_time*(x/2.)  
        for x in range(2,NX):
            V[1,x,Y] = 0.            
            V[1,x,X] = -U_time*((x-1)/2.)
        for y in range(1,NY):    #X has won
            V[y,0,X] = 0. 
            V[y,0,Y] = -U_time*(y/2.)  
        for y in range(2,NY):
            V[y,1,X] = 0.           
            V[y,1,Y] = -U_time*((y-1)/2.)
        #crash end states -- overwrite
        V[0,0,Y] = U_crash_Y
        V[0,0,X] = U_crash_X
        V[1,1,Y] = U_crash_Y
        V[1,1,X] = U_crash_X
        return V

    def solveGame(self,U_crash_Y=-100, U_crash_X=-100, U_time=1., NY=20, NX=20):

        V = self.valueMatrixEndCases(U_crash_Y, U_crash_X, U_time, NY, NX)

        #recursively compute optimal strategies and game values
        S = np.zeros((NY,NX,2))   #strategies at each point, as yield probs
        for x in range(2,NX):
            for y in range(2,NY):
                Y=[[0,0],[0,0]]
                X=[[0,0],[0,0]]

                #form the 2x2 subgame payoff matrix.  Actions: move-1, move-2
                for ay in [1,2]:        #action Y player, action X player
                    for ax in [1,2]:
                        y_next = y-ay
                        x_next = x-ax
                        val_y  = V[y_next, x_next, 0]   # store the value at this point in the matrix into a variable
                        val_x  = V[y_next, x_next, 1]

                        Y[ay-1][ax-1] = val_y-1     #each tick pays 1 second
                        X[ay-1][ax-1] = val_x-1

                G = nashpy.Game(Y, X)
                eqs = G.support_enumeration()   #computing the nashes with nashpy library  (finding Nash Equilibrium)
                eq_list=[]                      #convert to list format
                for eq in eqs:
                    eq_list.append(eq)          # store the nashes in the new list


    ############################################################################
                for eq in eq_list:
                        #pdb.set_trace()
                        b_sym = np.abs(eq[0][0] - eq[1][0]) < 0.0001            #is this one symetric?
                        if b_sym:
                                b_exists_sym_eq = True                          #if so, remember that a sym one exists


                #throw away any dominated ones
                eq_best = 0 
                vY_best = -999 #vals of both players at single best known equilibrium
                vX_best = -999
                for eq in eq_list:
                        b_sym = np.abs(eq[0][0]-eq[1][0]) < 0.0001 #real number equality is uncomputable!
                                                        #how do we know what floating point
                                                        #is being used by the other player !?

                        #if b_exists_sym_eq and not b_sym:
                        #	continue   #discard asym equilibria if there exist sym ones

                        (vY,vX) = G[eq[0], eq[1]]         #values to players
                        #b_dom = ( vY<vY_best and vX < vX_best )    #is it dominated?

                        #HACK HACK HACK  -- pick out the one which is symetric
                        #if not b_dom:    #assume for now that there is only one of these!
                        if b_sym:    #assume for now that there is only one of these!
                                eq_best = eq     #TODO metatrategy convergence is more than one
                                vY_best=vY
                                vX_best=vX

                #then meta-strategy convergence... TODO
    ############################################################################


    #            pdb.set_trace()
                ##(eq_best, vY_best, vX_best) = solutionConcepts.selectEquilibrium(eq_list, G)
        
                #log the results
                S[y,x,0] = eq_best[0][0]  #yield probs -> strategy  
                S[y,x,1] = eq_best[1][0]
                V[y,x,0] = vY_best
                V[y,x,1] = vX_best
        return (V,S)


    def sim(self,S, ystart, xstart, seed=0):  #S=strategy matrix; y,x start locations
        random.seed(seed)
        y=ystart
        x=xstart
        ypos=[y]
        xpos=[x]
        done=False
        while not done:
            p_yield_y = S[y,x,0]    
            p_yield_x = S[y,x,1]
            r1=random.random()
            r2=random.random()
            if r1 < p_yield_y:
                ay=1
            else:
                ay=2
            if r2 < p_yield_x:
                ax=1
            else:
                ax=2
            y-=ay
            x-=ax
            ypos.append(y)
            xpos.append(x)
            if y<2 or x<2:
                done=True
        clf()
        plot( range(0, len(ypos)), ypos, 'k')
        plot( range(0, len(xpos)), xpos, 'k--')
        plot( [0, len(ypos)] , [0,0], 'k')
        legend(['y, Y position / meters', 'x, X position / meters'])
        xlabel('time')
        ylabel('vehicle location')
        title('Simulated trajectories')
        return (ypos, xpos)

    def computeStateProbs(self, S, y_init, x_init): #compute prob that a state is ever visited in a run
            P=np.zeros((S.shape[0], S.shape[1]))
            P[y_init, x_init] = 1.  #start state
            y=y_init
            while(y>1):
                    x=x_init
                    while(x>1):
                            p_y_1 = S[y,x,0]
                            p_y_2 = 1 - p_y_1
                            p_x_1 = S[y,x,1]
                            p_x_2 = 1 - p_x_1
                            P[y-1, x-1] += P[y,x]*p_y_1*p_x_1
                            P[y-1, x-2] += P[y,x]*p_y_1*p_x_2
                            P[y-2, x-1] += P[y,x]*p_y_2*p_x_1
                            P[y-2, x-2] += P[y,x]*p_y_2*p_x_2
                            x-=1
                    y-=1
            p_crash = P[0,0] + P[1,1]   #total prob of a crash occuring
            return (P,p_crash)

