  
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
    open_container = util.Stack()
    start = (problem.getStartState(), None, 0)
    
    lst = [start]
    open_container.push(lst)
    while not open_container.isEmpty():
        path = open_container.pop()
        end = path[-1][0]
        if problem.isGoalState(end):
            actions = []
            for (_,a,_) in path:
                actions.append(a)
            return actions[1:]
        for succ in problem.getSuccessors(end):
            past = []
            for (s,_,_) in path[:-1]:
                past.append(s)
            if succ[0] not in past:
                open_container.push(path + [succ])
    return list()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    open_container = util.Queue()
    start = (problem.getStartState(), None, 0)
    
    open_container.push([start])
    visited = {start[0]: 0}

    while not open_container.isEmpty():
        path = open_container.pop()
        end, _, _ = path[-1]
        actions = []
        for (_,a,_) in path[1:]:
            actions.append(a)
        cost = 0
        for (_,_,c) in path[1:]:
            cost += c

        if not (cost > visited[end]):
            if problem.isGoalState(end):
                return actions
            for succ in problem.getSuccessors(end):
                new_path = path + [succ]
                new_cost = 0
                for (_,_,c) in new_path:
                    new_cost += c
                state = succ[0]
                if (state not in visited) or (new_cost < visited[state]):
                    open_container.push(new_path)
                    visited[state] = new_cost
    return list()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    open_container = util.PriorityQueue()

    start = problem.getStartState()
    start_entry = (start, None, 0)
   
    open_container.push([start_entry], start_entry[2])
    visited = {start_entry[0]: start_entry[2]}

    while not open_container.isEmpty():
        path = open_container.pop()
        end = path[-1][0]
        actions = []
        for (_,a,_) in path[1:]:
            actions.append(a)
        cost = 0
        for (_,_,c) in path[1:]:
            cost += c
    
        if not(cost > visited[end]):
            if problem.isGoalState(end):
                return actions
            for succ in problem.getSuccessors(end):
                new_path = path + [succ]
                new_cost = 0
                for (_,_,c) in new_path:
                    new_cost += c
                state = succ[0]
                if (state not in visited) or (new_cost < visited[state]):
                    open_container.push(new_path, new_cost)
                    visited[state] = new_cost 
    return list()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    open_container = util.PriorityQueue()
    start = problem.getStartState()
    start_entry = (start, None, 0)
   
    #In OPEN, the priority we add is a tuple (cost + h, h), so whenever cos+h is same for elements, just h is checked.
    open_container.push([start_entry], (start_entry[2]+ heuristic(start,problem),heuristic(start,problem)))
    visited = {start_entry[0]: start_entry[2] + heuristic(start,problem)}

    while not open_container.isEmpty():
        path = open_container.pop()

        end = path[-1][0]
        cost = 0
        for (_,_,c) in path[1:]:
            cost += c
        actions = []
        for (_,a,_) in path[1:]:
            actions.append(a)
        
        if not(cost > visited[end]):
            if problem.isGoalState(end):
                return actions
            for succ in problem.getSuccessors(end):
                new_path = path + [succ]
                new_cost = 0
                for (_, _, c) in new_path:
                    new_cost += c
                state = succ[0]
                if (state not in visited) or (new_cost < visited[state]):
                    open_container.push(new_path, (new_cost + heuristic(state, problem), heuristic(state, problem)))
                    visited[state] = new_cost
    return list()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
