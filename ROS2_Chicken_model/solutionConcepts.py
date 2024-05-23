#!/usr/bin/env python3

import numpy as np
from pylab import *
import nashpy
import pygambit
import pdb
import random


class SolutionConcepts():

    def __init__(self):
        pass
    
    #for N equilibria, build an N*N bool matrix whose (i,j) says if ith equilibrium dominates the jth.
    def computeDominances(self, eq_list, G):
        #compute values of eqs to the two players
        N = len(eq_list)
        vals_Y = np.zeros(N)    #values to the row (Y=0) player
        vals_X = np.zeros(N)    #values to the column (X=1) player
        for i in range(0,N):
            eq = eq_list[i]
            (vY,vX) = G[eq[0], eq[1]]         #values to players, under this pair of stretegies
            vals_Y[i] = vY
            vals_X[i] = vX
        dominates = np.zeros((N,N))
        for i in range(0,N):
            for j in range(0,N):
                if (vals_Y[i]>vals_Y[j])  and (vals_X[i] > vals_X[j]):
                    dominates[i,j]=1
        return dominates

    def removeDominatedEquilibria(self, eq_list, G):
        dominates = self.computeDominances(eq_list, G)
        eq_list_pruned = []
        for j in range(0, len(eq_list)):
            if sum(dominates[:,j])==0:              #is the jth eq dominated by anyone? if not, then keep it
                eq_list_pruned.append(eq_list[j])
        return eq_list_pruned


    def equilibriumIsSymmetric(self, eq):    #eq is a pair of stragegies for the 2 players, with probs of the two actions each
            b_sym = np.abs(eq[0][0] - eq[1][0]) < 0.0001            #symetric
            return b_sym
            #NOTE real number equality is uncomputable! 
            #NOTE so how do we know what floating point is being used by the other player !?   

    def removeAsymmetricEquilibriaIfThereAreAnySymmetricEquilibria(self, eq_list):
        b_exists_sym_eq = False
        for eq in eq_list:
            if self.equilibriumIsSymmetric(eq):
                b_exists_sym_eq = True          
        if not b_exists_sym_eq:
            return eq_list   #do nothing
        else:
            eq_list_pruned = []
            for eq in eq_list:
                if self.equilibriumIsSymmetric(eq):
                    eq_list_pruned.append(eq)
            return eq_list_pruned

    def useTheFirstEquilibrium(self,eq_list):    #HACK until we have anything better
        return eq_list[0]

    def averageOfEquilibria(self,eq_list):   #each player picks an equilibrium at random with a flat prior then draws their action from it
        N = len(eq_list)                #NOTE the result might not be an equlibrium itself -- hence meta strategy convertgence to refine from it
        eq_best = [ np.array([0.,0.]) , np.array([0., 0.])  ]
        for eq in eq_list:
            eq_best[0] += eq[0]
            eq_best[1] += eq[1]
        eq_best[0]/=N
        eq_best[1]/=N
        return eq_best

    def metaStrategyConvergence(self,eq_list, G):
        #TODO magic goes here
        #start by asigning flat probs to each equilibrium. 
        #find total probs of the actions under this
        #the result is not (usually?) an equlibrium itself.
        #start at it, form a new game for players to select actions corresponding to eqlibrrium selection?
        #(we know that one exists because we started with a list of them)
        #ie we are asking, whose basin does the inital average lie in.
        return eq_list

    def selectEquilibrium(self, eq_list, G):   #G is the game representation, used to compute values for equilibrs
        eq_list = self.removeDominatedEquilibria(eq_list, G)  #TODO test
    #    eq_list = removeAsymmetricEquilibriaIfThereAreAnySymmetricEquilibria(eq_list) #TODO test
    #    eq_best = useTheFirstEquilibrium(eq_list)
        eq_best = self.averageOfEquilibria(eq_list)
    #    eq_best = metaStrategyConvergence(eq_list, G)
        (vY_best,vX_best) = G[eq_best[0], eq_best[1]]         #values to players, under this pair of stretegies
        return (eq_best, vY_best, vX_best) 

