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
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState() -->start point(9,1)
    print "Is the start a goal?", problem.isGoalState(problem.getStartState()) False
    print "Start's successors:", problem.getSuccessors(problem.getStartState()) [((10, 1), 'East', 1), ((8, 1), 'West', 1)]

    """
    "*** YOUR CODE HERE ***"
    from util import Stack

    #initialise
    fringe = Stack()
    label_visted = []
    direction = []
    start_point = problem.getStartState()
    label_visted.append(start_point)
    fringe.push((start_point,direction,label_visted))

    #if stack is not empty, continue to check the next point, else return the action list
    while fringe.isEmpty() == False or problem.isGoalState(start_point) ==False:
        (start_point, direction,label_visted) = fringe.pop()


        # if the start point is goal, end the loop and return the action list
        if problem.isGoalState(start_point) ==True:
            return direction
            break

        else:
            next_point = problem.getSuccessors(start_point)
            temp_dir = direction
            temp_vist = label_visted
            for i in next_point:
                # check whether this new point already visted or not
                if ( i[0] not in label_visted):
                    direction = temp_dir
                    direction = direction + [i[1]]
                    label_visted = temp_vist
                    label_visted = label_visted + [i[0]]
                    fringe.push((i[0],direction,label_visted))


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first.
        python pacman.py -p SearchAgent -a fn=breadthFirstSearch
        python autograder.py

    """
    "*** YOUR CODE HERE ***"
    from util import Queue
    fringe = Queue()
    label_visted = []
    direction = []
    start_point = problem.getStartState()
    label_visted.append(start_point)
    nbhd = []

    fringe.push((start_point,direction,label_visted))

    while fringe.isEmpty() == False or problem.isGoalState(start_point) == False :
        (start_point,direction,label_visted)= fringe.pop()

        if start_point not in nbhd:
            nbhd.append(start_point)
            temp_dir = direction
            temp_vist = label_visted

            if(problem.isGoalState(start_point) == True):
                return direction
                break

            next_point = problem.getSuccessors(start_point)
            for i in next_point:
                if i[0] not in label_visted:
                    direction = temp_dir
                    direction = direction + [i[1]]
                    label_visted = temp_vist
                    label_visted = label_visted + [i[0]]
                    fringe.push((i[0], direction , label_visted))


def uniformCostSearch(problem):
    """Search the node of least total cost first.
    python pacman.py -p SearchAgent -a fn=uniformCostSearch
            python autograder.py

    """
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    fringe = PriorityQueue()
    direction = []
    accumulate_cost= 0
    nbhd = []
    start_point = problem.getStartState()
    label_visted = []
    label_visted.append(start_point)

    fringe.push((start_point,direction,accumulate_cost,nbhd,label_visted),accumulate_cost)


    while fringe.isEmpty() == False or problem.isGoalState(start_point) == False:
        (start_point,direction,accumulate_cost,nbhd,label_visted)=fringe.pop()

        if (start_point not in nbhd):
            nbhd.append(start_point)
            temp_cost = accumulate_cost
            temp_dir = direction
            temp_vist = label_visted


            if (problem.isGoalState(start_point) == True):
                return direction
                break

            else:
                next_point = problem.getSuccessors(start_point)
                for i in next_point:
                    if i[0] not in label_visted:
                        label_visted = temp_vist
                        label_visted = label_visted + [i[0]]
                        accumulate_cost =temp_cost
                        accumulate_cost = accumulate_cost+i[2]
                        direction = temp_dir
                        direction = direction + [i[1]]
                        fringe.push((i[0],direction,accumulate_cost,nbhd,label_visted),accumulate_cost)

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    from util import PriorityQueue
    fringe = PriorityQueue()
    direction = []
    accumulate_cost = 0
    nbhd = []
    start_point = problem.getStartState()
    h = heuristic(start_point,problem)
    newAccu_cost = accumulate_cost + h
    label_visted = []
    label_visted.append(start_point)
    fringe.push((start_point, direction, accumulate_cost, nbhd,label_visted), newAccu_cost)

    while fringe.isEmpty() == False:
        (start_point, direction, accumulate_cost,nbhd,label_visted) = fringe.pop()

        if (start_point not in nbhd):
            nbhd.append(start_point)
            temp_cost = accumulate_cost
            temp_dir = direction
            temp_vist = label_visted


            if (problem.isGoalState(start_point) == True):
                return direction
                break

            else:
                next_point = problem.getSuccessors(start_point)
                for i in next_point:
                    if i[0] not in label_visted:
                        label_visted = temp_vist
                        label_visted = label_visted + [i[0]]
                        accumulate_cost = temp_cost
                        accumulate_cost = accumulate_cost + i[2]
                        newAccu_cost = accumulate_cost + heuristic(i[0], problem)
                        direction = temp_dir
                        direction = direction + [i[1]]
                        fringe.push((i[0], direction, accumulate_cost,nbhd,label_visted),
                                    newAccu_cost)



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
