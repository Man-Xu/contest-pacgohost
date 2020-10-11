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
from captureAgents import CaptureAgent
import copy
import random, util
from game import Directions
from util import nearestPoint
from collections import defaultdict


#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'OffensiveReflexAgent', second = 'DefensiveReflexAgent'):
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """

  # The following line is an example only; feel free to change it.
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########
class OffensiveReflexAgent(CaptureAgent):
  """
  An agent only need to attack
  """
  def registerInitialState(self, gameState):
    CaptureAgent.registerInitialState(self, gameState)
    self.initialState = gameState
    self.goal = None
    self.midX = int((gameState.data.layout.width - 2) / 2)
    self.height = gameState.data.layout.height
    self.oppoMidX = int((gameState.data.layout.width - 2) / 2)
    self.duplicateMove = 0
    self.step = 0
    self.safeDist = 8

    if not self.red:
      self.midX += 1
    else:
      self.oppoMidX += 1

    self.previousFoodlist = []
    self.findAttackPos(gameState)
    self.attackPosRecord = defaultdict(int)

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
    myPos = gameState.getAgentPosition(self.index)
    actions = gameState.getLegalActions(self.index)
    opponentsIndex = self.getOpponents(gameState)
    capsuleList = self.getCapsules(gameState)
    actions.remove(Directions.STOP)
    self.step += 1

    revDir = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
    preDir = None
    if self.getPreviousObservation():
      preDir = self.getPreviousObservation().getAgentState(self.index).configuration.direction

    if preDir == revDir:
      self.duplicateMove += 1
    else:
      self.duplicateMove = 0
    if self.duplicateMove >= 5:
      actions.remove(revDir)

    if not gameState.getAgentState(self.index).isPacman:
      self.foodLeft = len(self.getFood(gameState).asList())
      """
      on my side
      """
      if self.goal == None or self.goal not in self.attackPosList or (myPos in self.attackPosList and self.step - self.attackPosRecord[myPos] <= 10):
        self.goal = random.choice(self.attackPosList)
      if myPos in self.attackPosList and self.step - self.attackPosRecord[myPos] > 10:
        """
        attemp to attack
        """
        self.attackPosRecord[myPos] = self.step

        attackDir = Directions.WEST
        if self.red:
          attackDir = Directions.EAST
        opponentsPos = []
        bestDist = 99999
        for oppoIndex in opponentsIndex:
          opponentState = gameState.getAgentState(oppoIndex)
          oppoPos = opponentState.getPosition()
          if oppoPos != None:
            opponentsPos.append(oppoPos)
            bestDist = min(bestDist, self.getMazeDistance(myPos, oppoPos))
          if len(opponentsPos) == 0 or bestDist >= 5:
            return attackDir

          successor = self.getSuccessor(gameState, attackDir)
          sucPos = successor.getAgentState(self.index).getPosition()
          for oppoPos in opponentsPos:
            if self.getMazeDistance(sucPos, oppoPos) <= 3 or self.getMazeDistance(sucPos, oppoPos) > 10:
              restAttackPosList = copy.deepcopy(self.attackPosList)
              restAttackPosList.remove(myPos)
              self.goal = random.choice(restAttackPosList)
              return self.findAction(gameState, actions)
          return attackDir
        else:
          return self.findAction(gameState, actions)
      else:
        """
        on opponent side
        """
        foodList = self.getFood(gameState).asList()
        self.foodCarry = self.foodLeft - len(foodList)

        opponentsPos = []
        bestDist = 99999
        for oppoIndex in opponentsIndex:
          opponentState = gameState.getAgentState(oppoIndex)
          oppoPos = opponentState.getPosition()
          if oppoPos != None and not gameState.getAgentState(oppoIndex).isPacman:
            if bestDist > self.getMazeDistance(myPos, oppoPos):
              bestDist = self.getMazeDistance(myPos, oppoPos)
              closestIndex = oppoIndex
            opponentsPos.append(oppoPos)

        bestDistHome = 99999
        for goal in self.attackPosList:
          if bestDistHome > self.getMazeDistance(goal, myPos):
            bestDistHome = self.getMazeDistance(goal, myPos)
            homeGoal = goal

        if bestDistHome > gameState.data.timeleft/4 - 2 or len(foodList) == 0:
          self.goal = homeGoal
          return  self.findAction(gameState, actions, False)

        if len(opponentsPos) == 0 or bestDist > self.safeDist:
          """
          When it is safe to find food
          """
          if self.getScore(gameState) >= 1:
            action, destination, cost = self.aStarSearch(gameState, 1)
          else:
            action, destination, cost = self.aStarSearch(gameState, 2)
          return action

        """
        When I have eaten the capsule
        """
        availableTime = gameState.getAgentState(closestIndex).scaredTimer
        oppoPos = gameState.getAgentState(closestIndex).getPosition()
        if availableTime > 0:
          action, destination, cost = self.aStarSearch(gameState, 2)

          safeGoal = self.attackPosList + capsuleList
          bestDist = 99999
          for goal in safeGoal:
            if bestDist > self.getMazeDistance(goal, destination):
              bestDist = self.getMazeDistance(goal, destination)
              bestGoal = goal

          bestDistHome = 99999
          for goal in self.attackPosList:
            if bestDistHome > self.getMazeDistance(goal, destination):
              bestDistHome = self.getMazeDistance(goal, destination)
              homeGoal = goal

          if (bestDistHome + cost) > gameState.data.timeleft/4 - 1:
            self.goal = homeGoal
            return self.findAction(gameState, actions, False)

          if (bestDist + cost) < availableTime:
            return action
          else:
            self.goal = bestGoal
            return self.findAction(gameState, actions, False)

        if self.foodCarry >= 2:
          self.goals = self.attackPosList + capsuleList
        else:
          if len(capsuleList) > 0:
            self.goals = capsuleList
            ans = self.BrFS(gameState, oppoPos)
            if ans == Directions.STOP:
              self.goals = self.attackPosList
              self.safeDist = max(2, self.safeDist - 0.25)
          else:
            self.goals = self.attackPosList
            self.safeDist = max(2, self.safeDist - 0.25)

        return self.BrFS(gameState, oppoPos)

  def findAction(self, gameState, actions, flag=True):
    bestAction = None
    bestDist = 99999
    for action in actions:
      successor = self.getSuccessor(gameState, action)
      if flag and successor.getAgentState(self.index).isPacman:
        continue
      toPos = successor.getAgentPosition(self.index)
      dist = self.getMazeDistance(self.goal, toPos)
      if dist < bestDist:
        bestAction = action
        bestDist = dist
    if bestAction == None:
      bestAction = Directions.STOP
    return bestAction

  def aStarSearch(self, gameState, goal):
    openList = util.PriorityQueue()
    hasVisited = set()
    bestCost = {}
    goalLeft = goal
    initialFoodList = self.getFood(gameState).asList()
    if len(initialFoodList) == 1:
      goalLeft = 1
    goalLeft = max(1, goalLeft - self.foodCarry)
    pos = gameState.getAgentState(self.index).getPosition()
    bestCost[pos] = 0
    openList.push((gameState, None, 0), 0 + self.heuristic(gameState, goalLeft, initialFoodList))

    while not openList.isEmpty():
      state, solution, cost = openList.pop()
      currentFoodList = self.getFood(state).asList()

      pos = state.getAgentPosition(self.index)
      if pos not in hasVisited or cost < bestCost[pos]:
        hasVisited.add(pos)
        bestCost[pos] = cost
        foodEaten = len(initialFoodList) - len(currentFoodList)
        if foodEaten >= goalLeft:
          if solution == None:
            solution = Directions.STOP
          return (solution, pos, cost)
        actions = state.getLegalActions(self.index)
        for action in actions:
          sucState = self.getSuccessor(state, action)
          if not sucState.getAgentState(self.index).isPacman:
            continue
          if solution:
            openList.push((sucState, solution, cost + 1), cost + 1 + self.heuristic(sucState, goalLeft - foodEaten, currentFoodList))
          else:
            openList.push((sucState, action, cost + 1), cost + 1 + self.heuristic(sucState, goalLeft - foodEaten, currentFoodList))
    if goal > 0:
      return self.aStarSearch(gameState, goal - 1)
    else:
      actions = gameState.getLegalActions(self.index)
      return (random.choice(actions), pos, 0)

  def heuristic(self, gameState, goalNum, foodList):
      pos = gameState.getAgentPosition(self.index)
      bestDist1 = 99999
      bestDist2 = 99999
      for food in foodList:
        dist = self.getMazeDistance(pos, food)
        if dist < bestDist1:
          bestDist2 = bestDist1
          bestDist1 = dist
        elif dist < bestDist2:
          bestDist2 = dist
      if goalNum == 1:
        return bestDist1
      else:
        return bestDist1 + bestDist2

  def BrFS(self, gameState, oppoPos):
    queue = util.Queue()
    hasVisited = set()
    queue.push((gameState, None))
    while not queue.isEmpty():
      state, solution = queue.pop()
      pos = state.getAgentPosition(self.index)
      if pos not in hasVisited:
        hasVisited.add(pos)
        if pos in self.goal:
          if solution == None:
            solution = Directions.STOP
          return solution
      actions = state.getLegalActions(self.index)

      for action in actions:
        sucState = self.getSuccessor(state, action)
        sucPos = sucState.getAgentPosition(self.index)

        if self.getMazeDistance(sucPos, oppoPos) <= 1:
          continue
        if solution:
          queue.push((sucState, solution))
        else:
          queue.push((sucState, action))
    return Directions.STOP

class DefensiveReflexAgent(CaptureAgent):
  """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """

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
        while(len(eatenFood) != 0):
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
        if (self.red and self.goal[0] > self.midX) or ((not self.red) and self.goal[0] < self.midX) or (self.goal == None):
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

