# -*- coding:utf-8 -*-
import random, util
from util import manhattanDistance
from game import Directions
from game import Agent

# Example Agent
class ReflexAgent(Agent):

  def Action(self, gameState):
    move_candidate = gameState.getLegalActions()
    scores = [self.reflex_agent_evaluationFunc(gameState, action) for action in move_candidate]
    bestScore = max(scores)
    Index = [index for index in range(len(scores)) if scores[index] == bestScore]
    get_index = random.choice(Index)

    return move_candidate[get_index]

  def reflex_agent_evaluationFunc(self, currentGameState, action):
    successorGameState = currentGameState.generatePacmanSuccessor(action)
    newPos = successorGameState.getPacmanPosition()
    oldFood = currentGameState.getFood()
    newGhostStates = successorGameState.getGhostStates()
    newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

    return successorGameState.getScore()


def scoreEvalFunc(currentGameState):
  return currentGameState.getScore()


class AdversialSearchAgent(Agent):
  def __init__(self, getFunc ='scoreEvalFunc', depth ='2'):
    self.index = 0
    self.evaluationFunction = util.lookup(getFunc, globals())
    self.depth = int(depth)


# ----------------------- < MinimaxAgent > ----------------------- #
class MinimaxAgent(AdversialSearchAgent):

  def Action(self, gameState):

    # 1. Minimax-Decision function
    def minimax(gameState, depth, agentIndex):
      if agentIndex == gameState.getNumAgents():
        depth += 1; agentIndex = 0

      # Is-Terminal-Test
      if (depth == self.depth or gameState.isWin() or gameState.isLose()):
        return self.evaluationFunction(gameState)
      # Pacman
      elif (agentIndex == 0):
        return maxValue(gameState, depth, agentIndex)
      # Ghost
      else:
        return minValue(gameState, depth, agentIndex)

    # 2. Max-Value function
    def maxValue(gameState, depth, agentIndex):
      maxVal = float("-inf")
      legalActions = gameState.getLegalActions(agentIndex)

      for direction in legalActions:
        newState = gameState.generateSuccessor(agentIndex, direction)
        new = minimax(newState, depth, agentIndex + 1)
        if type(new) == list: # new = list : max/minValue return value
          newVal = new[1]
        else: # new = int : evaluationFunction return value
          newVal = new
        if newVal > maxVal:
          maxVal = newVal
          result = [direction, maxVal]

      return result

    # 3. Min-Value function
    def minValue(gameState, depth, agentIndex):
      minVal = float("inf")
      legalActions = gameState.getLegalActions(agentIndex)

      for direction in legalActions:
        newState = gameState.generateSuccessor(agentIndex, direction)
        new = minimax(newState, depth, agentIndex + 1)
        if type(new) == list: # new = list : max/minValue return value
          newVal = new[1]
        else: # new = int : evaluationFunction return value
          newVal = new
        if newVal < minVal:
          minVal = newVal
          result = [direction, minVal]

      return result

    # return result
    result = minimax(gameState, 0, 0)
    return result[0]


# ----------------------- < AlphaBetaAgent > ----------------------- #
class AlphaBetaAgent(AdversialSearchAgent):

  def Action(self, gameState):

    # 1. Minimax function with Alpha-beta Pruning
    def alphaBeta(gameState, depth, agentIndex, a, b):
      if agentIndex == gameState.getNumAgents():
        depth += 1; agentIndex = 0
        
      # Is-Terminal-Test
      if (depth == self.depth or gameState.isWin() or gameState.isLose()):
        return self.evaluationFunction(gameState)
      # Pacman
      elif (agentIndex == 0):
        return maxValue(gameState, depth, agentIndex, a, b)
      # Ghost
      else:
        return minValue(gameState, depth, agentIndex, a, b)

    # 2. Max-Value function
    def maxValue(gameState, depth, agentIndex, a, b):
      maxVal = float("-inf")
      legalActions = gameState.getLegalActions(agentIndex)

      for direction in legalActions:
        newState = gameState.generateSuccessor(agentIndex, direction)
        new = alphaBeta(newState, depth, agentIndex + 1, a ,b)
        if type(new) == list: # new = list : max/minValue return value
          newVal = new[1]
        else: # new = int : evaluationFunction return value
          newVal = new
        if newVal > maxVal:
          maxVal = newVal
          result = [direction, maxVal]
          a = max(a, newVal)

        # Alpha-beta Pruning
        if newVal > b:
          return [direction, newVal]

      return result

    # 3. Min-Value function
    def minValue(gameState, depth, agentIndex, a, b):
      minVal = float("inf")
      legalActions = gameState.getLegalActions(agentIndex)

      for direction in legalActions:
        newState = gameState.generateSuccessor(agentIndex, direction)
        new = alphaBeta(newState, depth, agentIndex + 1, a, b)
        if type(new) == list: # new = list : max/minValue return value
          newVal = new[1]
        else: # new = int : evaluationFunction return value
          newVal = new
        if newVal < minVal:
          minVal = newVal
          result = [direction, minVal]
          b = min(b, newVal)

        # Alpha-beta Pruning
        if newVal < a:
          return [direction, newVal]

      return result

    # return result
    alpha = float("-inf"); beta = float("inf")
    result = alphaBeta(gameState, 0, 0, alpha, beta)
    return result[0]


# ----------------------- < ExpectimaxAgent > ----------------------- #
class ExpectimaxAgent(AdversialSearchAgent):

  def Action(self, gameState):

    # 1. Expectimax function
    def expectimax(gameState, depth, agentIndex):
      if agentIndex == gameState.getNumAgents():
        depth += 1; agentIndex = 0
        
      # Is-Terminal-Test
      if (depth == self.depth or gameState.isWin() or gameState.isLose()):
        return self.evaluationFunction(gameState)
      # Pacman
      elif (agentIndex == 0):
        return maxValue(gameState, depth, agentIndex)
      # Ghost
      else:
        return expectValue(gameState, depth, agentIndex)

    # 2. Max-Value function
    def maxValue(gameState, depth, agentIndex):
      maxVal = float("-inf")
      legalActions = gameState.getLegalActions(agentIndex)

      for direction in legalActions:
        newState = gameState.generateSuccessor(agentIndex, direction)
        new = expectimax(newState, depth, agentIndex + 1)
        if type(new) == list: # new = list : max/expectValue return value
          newVal = new[1]
        else: # new = int : evaluationFunction return value
          newVal = new
        if newVal > maxVal:
          maxVal = newVal
          result = [direction, maxVal]
      return result

    # 3. Expect-Value function
    def expectValue(gameState, depth, agentIndex):
      expectVal = 0
      legalActions = gameState.getLegalActions(agentIndex)
      prob = 1 / len(legalActions)

      for direction in legalActions:
        newState = gameState.generateSuccessor(agentIndex, direction)
        new = expectimax(newState, depth, agentIndex + 1)
        if type(new) == list: # new = list : max/expectValue return value
          newVal = new[1]
        else: # new = int : evaluationFunction return value
          newVal = new
        expectVal += newVal * prob
        result = [direction, expectVal]
      return result

    result = expectimax(gameState, 0, 0)
    return result[0]