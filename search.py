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
from game import Directions

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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """

    startState = (problem.getStartState(), None, 0)
    stack = util.Stack()
    stack.push([startState])
    while not stack.isEmpty():
        path = stack.pop()
        (endState, _, _) = path[-1]
        if problem.isGoalState(endState):
            actions = [action for (_, action, _) in path]
            return actions[1:]
        for succ in problem.getSuccessors(endState):
            past = [state for (state, _, _) in path[:-1]]
            if succ[0] not in past:
                stack.push(path + [succ])
    return []


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    startState = (problem.getStartState(), None, 0)
    queue = util.Queue()
    queue.push([startState])
    seen = {startState[0]: 0}

    while not queue.isEmpty():
        path = queue.pop()
        (endState, _, _) = path[-1]
        actions = [action for (_, action, _) in path[1:]]
        cost = sum([c for (_, _, c) in path[1:]])

        if cost <= seen[endState]:
            if problem.isGoalState(endState):
                return actions
            for succ in problem.getSuccessors(endState):
                newPath = path + [succ]
                newCost = sum([c for (_, _, c) in newPath])
                state = succ[0]
                if state not in seen or newCost < seen[state]:
                    queue.push(newPath)
                    seen[state] = newCost
    return []


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    startState = problem.getStartState()
    starttuple = (startState, None, 0)
    priQueue = util.PriorityQueue()
    priQueue.push([starttuple], starttuple[2])
    seen = {starttuple[0]: starttuple[2]}

    while not priQueue.isEmpty():
        path = priQueue.pop()
        (endState, _, _) = path[-1]
        actions = [action for (_, action, _) in path[1:]]
        cost = sum([c for (_, _, c) in path[1:]])

        if cost <= seen[endState]:
            if problem.isGoalState(endState):
                return actions
            for succ in problem.getSuccessors(endState):
                newPath = path + [succ]
                newCost = sum([c for (_, _, c) in newPath])
                state = succ[0]
                if state not in seen or newCost < seen[state]:
                    priQueue.push(newPath, newCost)
                    seen[state] = newCost
    return []


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    open = util.PriorityQueue()
    initCost = problem.getCostOfActions([]) + heuristic(problem.getStartState(), problem)
    open.push(([problem.getStartState()], []), initCost)
    seen = {problem.getStartState(): initCost}
    while not open.isEmpty():
        n = open.pop()
        state = n[0][-1]
        if problem.getCostOfActions(n[1]) <= seen[state]:
            if problem.isGoalState(state):
                return n[1]
            for succ in problem.getSuccessors(state):
                cost = problem.getCostOfActions(n[1] + [succ[1]]) + heuristic(succ[0], problem)
                if succ[0] not in seen or cost < seen[succ[0]]:
                    open.push((n[0] + [succ[0]], n[1] + [succ[1]]), cost)
                    seen[succ[0]] = cost
    return False


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
