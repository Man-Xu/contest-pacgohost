# myTeam.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
#
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).
import math

from captureAgents import CaptureAgent
import random, time, util
from game import Directions
from game import Actions
import game

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'Attacker', second = 'Defenser'):
  # The following line is an example only; feel free to change it.
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########

class DummyAgent(CaptureAgent):

  def registerInitialState(self, gameState):

    self.start = gameState.getAgentPosition(self.index)
    CaptureAgent.registerInitialState(self, gameState)
    self.walls = gameState.getWalls().asList()
    self.beenEatenFood = []

    '''
    Your initialization code goes here, if you need any.
    '''
  def getMiddleLines(self,gameState):
    if self.red:
      middleLine = [((gameState.data.layout.width / 2) - 1, y) for y in range(0, gameState.data.layout.height)]
    else:
      middleLine = [(gameState.data.layout.width / 2, y) for y in range(0, gameState.data.layout.height)]
    availableMiddle = [a for a in middleLine if a not in self.walls]
    middleDis = [self.getMazeDistance(gameState.getAgentState(self.index).getPosition(), mi) for mi in
                 availableMiddle]
    closeMiddle = [m for m, d in zip(availableMiddle, middleDis) if d == min(middleDis)]
    return closeMiddle[0]

  def getOpponentMiddleLines(self,gameState):
    if self.red:
      middleLine = [((gameState.data.layout.width / 2) , y) for y in range(0, gameState.data.layout.height)]
    else:
      middleLine = [((gameState.data.layout.width / 2) - 1, y) for y in range(0, gameState.data.layout.height)]
    availableMiddle = [a for a in middleLine if a not in self.walls]
    middleDis = [self.getMazeDistance(gameState.getAgentState(self.index).getPosition(), mi) for mi in
                 availableMiddle]
    closeMiddle = [m for m, d in zip(availableMiddle, middleDis) if d == min(middleDis)]
    return closeMiddle[0]

  def getNearestFood(self,gameState):
    food = self.getFood(gameState).asList()
    foodDis = [self.getMazeDistance(gameState.getAgentState(self.index).getPosition(), dis) for dis in
                 food]
    nearestDis = [m for m, d in zip(food, foodDis) if d == min(foodDis)]
    if len(nearestDis) == 0 : return None
    return nearestDis[0]

  def getNearestProtectFood(self,gameState,pos):
    food = self.getFoodYouAreDefending(gameState).asList()
    foodDis = [self.getMazeDistance(pos, dis) for dis in
                 food]
    nearestDis = [m for m, d in zip(food, foodDis) if d == min(foodDis)]
    if len(nearestDis) == 0 : return None
    return nearestDis[0]

  def getNearestCapsule(self, gameState):
    Caps = self.getCapsules(gameState)
    if len(self.getCapsules(gameState))==0 : return -1
    capDis = [self.getMazeDistance(gameState.getAgentState(self.index).getPosition(), dis) for dis in
               Caps]
    nearestDis = [m for m, d in zip(Caps, capDis) if d == min(capDis)]
    return nearestDis[0]


  def aStar(self,state,gameState,heuristic,goal):
    priorityQueue = util.PriorityQueue()
    visitedState = []
    actions = []
    totalCostOfF = 0
    priorityQueue.push((state, actions),totalCostOfF)
    while not priorityQueue.isEmpty():
      currentState, actions = priorityQueue.pop()
      if (currentState == goal):
        if len(actions) == 0: return 'Stop'
        break
      if (currentState not in visitedState):
        successors = self.getSuccessors(currentState)
        visitedState.append(currentState)
        for successor in successors:
          totalCostOfF = (heuristic(successor[0], gameState)) + len(actions+[successor[1]])
          if (successor[0] not in visitedState):
            priorityQueue.push((successor[0], actions + [successor[1]]), totalCostOfF)
    return actions[0]

  def offensiveHeuristic(self, state, gameState):
    heuristics = []
    ghosts = self.getDefenders(gameState)
    if ghosts == None:
      return 0
    else:
      for o in ghosts:
        if self.getMazeDistance(state, o.getPosition()) < 3 and o.scaredTimer == 0:
          d = self.getMazeDistance(state, o.getPosition())
          heuristics.append(math.pow((d - 5), 8))
        else:
          heuristics.append(0)
      return max(heuristics)

  def aStarKeepdistance(self,state,gameState,heuristic,goal):
    dist = self.getMazeDistance(state, goal)
    priorityQueue = util.PriorityQueue()
    visitedState = []
    actions = []
    totalCostOfF = 0
    priorityQueue.push((state, actions),totalCostOfF)
    while not priorityQueue.isEmpty():
      currentState, actions = priorityQueue.pop()
      if (dist in range (1,2)):
        if len(actions) == 0: return 'Stop'
        break
      if (currentState not in visitedState):
        successors = self.getSuccessors(currentState)
        visitedState.append(currentState)
        for successor in successors:
          totalCostOfF = (heuristic(successor[0], gameState)) + len(actions+[successor[1]])
          if (successor[0] not in visitedState):
            priorityQueue.push((successor[0], actions + [successor[1]]), totalCostOfF)
    return actions[0]

  def defensiveHeuristic(self, state, gameState):
    heuristics = []
    myState = gameState.getAgentState(self.index)
    pacMen = self.getIndavers(gameState)
    if pacMen == None:
      return 0
    else:
      for o in pacMen:
        if self.getMazeDistance(state, o.getPosition()) < 3:
          d = self.getMazeDistance(state, o.getPosition())
          heuristics.append(math.pow((d - 5), 8))
        else:
          heuristics.append(0)
      return max(heuristics)


  def nullHeuristic(self,state,gameState):
    return 0

  def getDefenders(self,gameState):
    enemies=[gameState.getAgentState(o) for o in self.getOpponents(gameState)]
    defenders=[a for a in enemies if a.getPosition() != None and not a.isPacman and a.scaredTimer == 0]
    if len(defenders) == 0:
      return  None
    else:
      return defenders

  def getIndavers(self,gameState):
    enemies = [gameState.getAgentState(o) for o in self.getOpponents(gameState)]
    indavers = [a for a in enemies if a.getPosition() != None and a.isPacman]
    if len(indavers) == 0:
      return None
    else:
      return indavers

  def realGetIndavers(self,gameState):
    enemies = [gameState.getAgentState(o) for o in self.getOpponents(gameState)]
    realIndavers = [a for a in enemies if a.isPacman]
    if len(realIndavers) == 0:
      return None
    else:
      return realIndavers

  def getProDefenders(self,gameState):
    enemies=[gameState.getAgentState(o) for o in self.getOpponents(gameState)]
    defenders=[a for a in enemies if not a.isPacman and a.scaredTimer == 0]
    if len(defenders) == 0:
      return  None
    else:
      return defenders

  def getSuccessors(self, state):
    successors = []
    wall = self.walls
    for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
      x, y = state
      dx, dy = Actions.directionToVector(action)
      nextx, nexty = int(x + dx), int(y + dy)
      if (nextx, nexty) not in wall:
        nextState = (nextx, nexty)
        successors.append((nextState, action))
    return successors

class Attacker(DummyAgent):
    def chooseAction(self, gameState):
        myState = gameState.getAgentState(self.index)
        myPos = gameState.getAgentState(self.index).getPosition()
        middle = self.getMiddleLines(gameState)
        opMiddle = self.getOpponentMiddleLines(gameState)
        nearestFood = self.getNearestFood(gameState)
        numCarry = gameState.getAgentState(self.index).numCarrying
        nearestCap = self.getNearestCapsule(gameState)
        ghosts = self.getDefenders(gameState)
        invanders = self.getIndavers(gameState)
        enemies = [gameState.getAgentState(o) for o in self.getOpponents(gameState)]
        unseenInvaders = [a for a in enemies if a.getPosition() != None and a.isPacman]
        carriedFoodOfEnemy = sum([a.numCarrying for a in unseenInvaders])
        currentFood = self.getFoodYouAreDefending(self.getCurrentObservation()).asList()

        if (carriedFoodOfEnemy > 8 and not myState.isPacman):
            if (invanders != None):
                enemyDis = [self.getMazeDistance(myPos, inv.getPosition()) for inv in
                            invanders]
                target = [m.getPosition() for m, d in zip(invanders, enemyDis) if d == min(enemyDis)]
                if myState.scaredTimer == 0:
                    return self.aStar(myPos, gameState, self.offensiveHeuristic, target[0])
                else:
                    return self.aStarKeepdistance(myPos, gameState, self.defensiveHeuristic, target[0])
            if (self.getPreviousObservation() != None and invanders == None):
                previousFood = self.getFoodYouAreDefending(self.getPreviousObservation()).asList()
                eatenFood = [food for food in previousFood if food not in currentFood]
                self.beenEatenFood += eatenFood
                if (len(self.beenEatenFood) > 0):
                    nearestProtectFood = self.getNearestProtectFood(self.getCurrentObservation(),
                                                                    self.beenEatenFood[-1])
                    action = self.aStar(myPos, gameState, self.defensiveHeuristic, nearestProtectFood)
                    if (action == 'Stop'): self.beenEatenFood = []
                    return action

        if (myPos == opMiddle and myState.numCarrying > 0):
            return self.aStar(myPos, gameState, self.nullHeuristic, middle)

        if (ghosts != None):
            if (nearestCap != -1 and myState.isPacman):
                return self.aStar(myPos, gameState, self.offensiveHeuristic, nearestCap)
            elif (numCarry > 0):
                return self.aStar(myPos, gameState, self.offensiveHeuristic, middle)

        if (self.getScore(gameState)) < 0:
            if (numCarry >= 4 or (self.getScore(gameState) + numCarry) > 0):
                return self.aStar(myPos, gameState, self.offensiveHeuristic, middle)

        if (numCarry > 8):
            return self.aStar(myPos, gameState, self.offensiveHeuristic, middle)

        return self.aStar(myPos, gameState, self.offensiveHeuristic, nearestFood)


class Defenser(DummyAgent):
  def chooseAction(self, gameState):
    myState = gameState.getAgentState(self.index)
    myPos = myState.getPosition()
    middle = self.getMiddleLines(gameState)
    invanders = self.getIndavers(gameState)
    currentFood = self.getFoodYouAreDefending(self.getCurrentObservation()).asList()
    realIndavers = self.realGetIndavers(gameState)
    nearestFood = self.getNearestFood(gameState)
    numCarry = gameState.getAgentState(self.index).numCarrying
    ghosts = self.getDefenders(gameState)

    if (realIndavers == None):
      if numCarry <3:
        if (ghosts != None):
          for g in ghosts:
            if (self.getMazeDistance(myPos, g.getPosition())<5):
              return self.aStar(myPos, gameState, self.offensiveHeuristic, middle)
        return self.aStar(myPos, gameState, self.offensiveHeuristic, nearestFood)
    if (invanders != None):
      enemyDis = [self.getMazeDistance(myPos, inv.getPosition()) for inv in
                  invanders]
      target = [m.getPosition() for m, d in zip(invanders, enemyDis) if d == min(enemyDis)]
      if myState.scaredTimer == 0:
        return self.aStar(myPos, gameState, self.offensiveHeuristic, target[0])
      else:
        return self.aStarKeepdistance(myPos, gameState, self.defensiveHeuristic, target[0])
    if (self.getPreviousObservation() != None and invanders == None):
      previousFood = self.getFoodYouAreDefending(self.getPreviousObservation()).asList()
      eatenFood = [food for food in previousFood if food not in currentFood]
      self.beenEatenFood += eatenFood
      if (len(self.beenEatenFood) > 0):
        nearestProtectFood = self.getNearestProtectFood(self.getCurrentObservation(), self.beenEatenFood[-1])
        action = self.aStar(myPos, gameState, self.defensiveHeuristic, nearestProtectFood)
        if (action == 'Stop'): self.beenEatenFood = []
        return action


    return self.aStar(myPos, gameState, self.defensiveHeuristic, middle)