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
from util import nearestPoint

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

    CaptureAgent.registerInitialState(self, gameState)
    self.start = gameState.getAgentPosition(self.index)
    self.walls = gameState.getWalls().asList()
    self.beenEatenFood = []

    self.middleLines = []
    self.goal = None
    self.midX = int((gameState.data.layout.width - 2) / 2)
    self.oppoMidX = int((gameState.data.layout.width - 2) / 2)
    self.opponentsIndex = self.getOpponents(gameState)
    if not self.red:
      self.midX += 1
    else:
      self.oppoMidX += 1

    self.attackDir = Directions.WEST
    if self.red:
        self.attackDir = Directions.EAST

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



  def findAction(self, gameState, actions):
    bestAction = None
    bestDist = 99999
    for action in actions:
      successor = self.getSuccessor(gameState, action)
      if successor.getAgentState(self.index).isPacman:
        continue
      toPos = successor.getAgentPosition(self.index)
      dist = self.getMazeDistance(self.goal, toPos)
      if dist < bestDist:
        bestAction = action
        bestDist = dist
    return bestAction

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
    if len(nearestDis) == 0: return None
    return nearestDis[0]

  def getNearestCapsule(self, gameState):
    Caps = self.getCapsules(gameState)
    if len(self.getCapsules(gameState))==0: return -1
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

  def hasIndavers(self,gameState):
    hasInvader = False
    for oppoIndex in self.opponentsIndex:
      if gameState.getAgentState(oppoIndex).isPacman:
        hasInvader = True
    return hasInvader

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
  def registerInitialState(self, gameState):
    """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).

    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)

    IMPORTANT: This method may run for at most 15 seconds.
    """

    '''
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.registerInitialState in captureAgents.py.
    '''
    CaptureAgent.registerInitialState(self, gameState)
    self.initialState = gameState
    self.goal = None
    self.midX = int((gameState.data.layout.width - 2) / 2)
    self.oppoMidX = int((gameState.data.layout.width - 2) / 2)
    if not self.red:
      self.midX += 1
    else:
      self.oppoMidX += 1

    self.previousFoodList = []
    self.findAttackPos(gameState)
    self.attackDir = Directions.WEST
    if self.red:
      self.attackDir = Directions.EAST

  def findAttackPos(self, gameState):
    self.attackPosList = []
    for i in range(1, gameState.data.layout.height - 1):
      if not gameState.hasWall(self.midX, i) and not gameState.hasWall(self.oppoMidX, i):
        self.attackPosList.append((self.midX, i))

  def getSuccessor(self, gameState, action):
    """
    Finds the next successor which is a grid position (location tuple).
    """
    successor = gameState.generateSuccessor(self.index, action)
    pos = successor.getAgentState(self.index).getPosition()
    if pos != nearestPoint(pos):
      # Only half a grid position was covered
      return successor.generateSuccessor(self.index, action)
    else:
      return successor

  def chooseAction(self, gameState):
      CaptureAgent.registerInitialState(self, gameState)
      self.initialState = gameState
      self.goal = None
      self.midX = int((gameState.data.layout.width - 2) / 2)
      self.oppoMidX = int((gameState.data.layout.width - 2) / 2)
      if not self.red:
          self.midX += 1
      else:
          self.oppoMidX += 1

      self.previousFoodList = []
      self.findAttackPos(gameState)
      self.attackDir = Directions.WEST
      if self.red:
          self.attackDir = Directions.EAST

  def findAttackPos(self, gameState):
      self.attackPosList = []
      for i in range(1, gameState.data.layout.height - 1):
          if not gameState.hasWall(self.midX, i) and not gameState.hasWall(self.oppoMidX, i):
              self.attackPosList.append((self.midX, i))

  def getSuccessor(self, gameState, action):
      """
      Finds the next successor which is a grid position (location tuple).
      """
      successor = gameState.generateSuccessor(self.index, action)
      pos = successor.getAgentState(self.index).getPosition()
      if pos != nearestPoint(pos):
          # Only half a grid position was covered
          return successor.generateSuccessor(self.index, action)
      else:
          return successor

  def chooseAction(self, gameState):
      """
      Picks among actions randomly.
      """
      currentFoodList = self.getFoodYouAreDefending(gameState).asList()
      myPos = gameState.getAgentPosition(self.index)
      actions = gameState.getLegalActions(self.index)
      opponentsIndex = self.getOpponents(gameState)
      hasInvader = False

      if myPos in self.attackPosList:
          actions.remove(self.attackDir)
      """
      If no invader on this side
      """
      for oppoIndex in opponentsIndex:
          if gameState.getAgentState(oppoIndex).isPacman:
              hasInvader = True

      if not hasInvader:
          if self.goal == myPos:
              self.goal = None
          if self.goal == None or myPos[0] == self.midX:
              self.goal = random.choice(self.attackPosList)
          return self.findAction(gameState, actions)
      else:
          invaderPos = []
          for oppoIndex in opponentsIndex:
              opponentState = gameState.getAgentState(oppoIndex)
              oppoPos = opponentState.getPosition()
              if opponentState.isPacman and oppoPos != None:
                  invaderPos.append(oppoPos)
          if len(currentFoodList) < len(self.previousFoodList):
              eatenFood = set(self.previousFoodList) - set(currentFoodList)
              while (len(eatenFood) != 0):
                  pos = eatenFood.pop()
                  if pos not in invaderPos:
                      invaderPos.append(pos)
          self.previousFoodList = currentFoodList
          """
          If I can find the invader's positon
          """
          bestDist = 99999
          bestPos = None
          if len(invaderPos) > 0:
              for pos in invaderPos:
                  distance = self.getMazeDistance(pos, myPos)
                  if distance < bestDist:
                      bestDist = distance
                      bestPos = pos
              self.goal = bestPos
              if gameState.getAgentState(self.index).scaredTimer > 0:
                  if bestDist < 3:
                      oppoDist = []
                      for action in actions:
                          sucState = gameState.generateSuccessor(self.index, action)
                          if sucState.getAgentState(self.index).isPacman:
                              oppoDist.append(0)
                              continue
                          sucPos = sucState.getAgentPosition(self.index)
                          dist = self.getMazeDistance(sucPos, self.goal)
                          if dist > 5:
                              dist = 0
                          oppoDist.append(dist)
                      bestValue = max(oppoDist)
                      bestAction = random.choice([a for a, v in zip(actions, oppoDist) if v == bestValue])
                      return bestAction
                  elif bestDist == 3:
                      return Directions.STOP
              return self.BrFS(gameState)
          else:
              """
              If I can't find the invader's positon,
              find a random food left on my side
              """
              if (self.red and self.goal[0] > self.midX) or ((not self.red) and self.goal[0] < self.midX) or (
                      self.goal == None):
                  self.goal = random.choice(self.getFoodYouAreDefending(gameState).asList())
              return self.findAction(gameState, actions)

  """
  find the best action in all alternative acitons
  by calculating the min maze distance
  """

  def findAction(self, gameState, actions):
      bestAction = None
      bestDist = 99999
      for action in actions:
          successor = self.getSuccessor(gameState, action)
          if successor.getAgentState(self.index).isPacman:
              continue
          toPos = successor.getAgentPosition(self.index)
          dist = self.getMazeDistance(self.goal, toPos)
          if dist < bestDist:
              bestAction = action
              bestDist = dist
      return bestAction

  def BrFS(self, gameState):
      queue = util.Queue()
      hasVisited = set()
      queue.push((gameState, None))
      while not queue.isEmpty():
          state, solution = queue.pop()
          pos = state.getAgentPosition(self.index)
          if pos not in hasVisited:
              hasVisited.add(pos)
              if pos == self.goal:
                  if solution == None:
                      solution = Directions.STOP
                  return solution
          actions = state.getLegalActions(self.index)

          if pos in self.attackPosList:
              actions.remove(self.attackDir)
          for action in actions:
              sucState = self.getSuccessor(state, action)
              sucPos = sucState.getAgentPosition(self.index)
              if (self.red and sucPos[0] > self.midX) or ((not self.red) and sucPos[0] < self.midX):
                  continue
              if self.getMazeDistance(pos, sucPos) > 1:
                  if solution:
                      return solution
                  else:
                      return action
              if solution:
                  queue.push((sucState, solution))
              else:
                  queue.push((sucState, action))
