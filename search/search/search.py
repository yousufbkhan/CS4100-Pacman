# search.py
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


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    frontier = util.Stack()                                         # create frontier of data type stack
    frontier.push((problem.getStartState(), [], 1))                 # add starting state to frontier
    explored = []                                                   # initialize explored set

    while not frontier.isEmpty():                                   # loop until frontier empty
        node = frontier.pop()                                       # pop the top most node on the stack
        currentState = node[0]                                      # get the state of the first node
        currentPath = node[1]                                       # get the path of the first node
        if problem.isGoalState(currentState):                       # return the path if this is the goal state
            return currentPath
        if currentState not in explored:                            # if this node has not been explored then:
            explored.append(currentState)                           # add node to explored set
            successors = problem.getSuccessors(currentState)        # get the successors of the current node
            for nextNode in successors:                             # iterate over the successors
                nextNodeState = nextNode[0]                         # get next node state
                nextNodePath = nextNode[1]                          # get next node path
                if nextNodeState not in explored:                   # if next node not in explored set:
                    nextNodePath = currentPath + [nextNodePath]     # add the next node path to the current path
                    frontier.push((nextNodeState, nextNodePath, 1)) # push the next node data to the frontier stack

def breadthFirstSearch(problem):
    frontier = util.Queue()                                         # create frontier of data type stack
    frontier.push((problem.getStartState(), [], 1))                 # add starting state to frontier
    explored = []                                                   # initialize explored set

    while not frontier.isEmpty():                                   # loop until frontier empty
        node = frontier.pop()                                       # pop first node on the Queue
        currentState = node[0]                                      # get the state of the first node
        currentPath = node[1]                                       # get the path of the first node

        if problem.isGoalState(currentState):                       # return the path if this is the goal state
            return currentPath

        if currentState not in explored:                            # if this node has not been explored then:
            explored.append(currentState)                           # add node to explored set
            successors = problem.getSuccessors(currentState)        # get the successors of the current node

            for nextNode in successors:                             # iterate over the successors
                nextNodeState = nextNode[0]                         # get next node state
                nextNodePath = nextNode[1]                          # get next node path
                if nextNodeState not in explored:                   # if next node not in explored set:
                    nextNodePath = currentPath + [nextNodePath]     # add the next node path to the current path
                    frontier.push((nextNodeState, nextNodePath, 1)) # push the next node data to the frontier stack

def uniformCostSearch(problem):
    frontier = util.PriorityQueue()                                         # create frontier of data type Priority Queue
    frontier.push((problem.getStartState(), [], 0), nullHeuristic(problem.getStartState(), problem)) # add starting state to frontier
    explored = []                                                           # initialize explored set

    while not frontier.isEmpty():                                           # loop until frontier empty
        node = frontier.pop()                                               # pop first node on the Queue
        currentState = node[0]                                              # get the state of the first node
        currentPath = node[1]                                               # get the path of the first node

        if problem.isGoalState(currentState):                               # return the path if this is the goal state
            return currentPath

        if currentState not in explored:                                    # if this node has not been explored then:
            explored.append(currentState)                                   # add node to explored set
            successors = problem.getSuccessors(currentState)                # get the successors of the current node

            for nextNode in successors:                                     # iterate over the successors
                nextNodeState = nextNode[0]                                 # get next node state
                nextNodePath = nextNode[1]                                  # get next node path
                if nextNodeState not in explored:                           # if next node not in explored set:
                    nextNodePath = currentPath + [nextNodePath]             # add the next node path to the current path
                    nextNodeCost = problem.getCostOfActions(nextNodePath)   # get cost of next node path
                    frontier.push((nextNodeState, nextNodePath, 0), nextNodeCost)  # push the next node data to the frontier stack

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""

    frontier = util.PriorityQueue()
    frontier.push((problem.getStartState(), []), heuristic(problem.getStartState(), problem))
    explored = []
    while not frontier.isEmpty():
        node = frontier.pop()
        currentState = node[0]
        currentPath = node[1]
        if problem.isGoalState(currentState):
            return currentPath
        if currentState not in explored:
            explored.append(currentState)
            successors = problem.getSuccessors(currentState)
            for nextNode in successors:
                nextNodeState = nextNode[0]
                nextNodePath = nextNode[1]
                if nextNodeState not in explored:
                    nextNodePath = currentPath + [nextNodePath]
                    nextNodeCost = problem.getCostOfActions(nextNodePath)
                    frontier.push((nextNodeState, nextNodePath), nextNodeCost + heuristic(nextNodeState, problem))

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
