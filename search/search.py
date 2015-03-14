# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
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
     Returns the start state for the search problem 
     """
     util.raiseNotDefined()
    
  def isGoalState(self, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     util.raiseNotDefined()

  def getSuccessors(self, state):
     """
       state: Search state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     util.raiseNotDefined()

  def getCostOfActions(self, actions):
     """
      actions: A list of actions to take
 
     This method returns the total cost of a particular sequence of actions.  The sequence must
     be composed of legal moves
     """
     util.raiseNotDefined()
           

def tinyMazeSearch(problem):
  """
  Returns a sequence of moves that solves tinyMaze.  For any other
  maze, the sequence of moves will be incorrect, so only use this for tinyMaze
  """
  from game import Directions
  s = Directions.SOUTH
  w = Directions.WEST
  return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
  """
  Search the deepest nodes in the search tree first [p 85].
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm [Fig. 3.7].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"
  fringe = util.Stack();
  expanded = set();
  fringe.push((problem.getStartState(),[],0));
  
  while not fringe.isEmpty():
        curState, curMoves, curCost = fringe.pop();

        if(curState in expanded or curState in fringe):
            continue;

        expanded.add(curState);

        if problem.isGoalState(curState):
            return curMoves;

        for state, direction, cost in problem.getSuccessors(curState):
            fringe.push((state, curMoves+[direction], curCost));            
  return [];
  

def breadthFirstSearch(problem):
  "Search the shallowest nodes in the search tree first. [p 81]"
  "*** YOUR CODE HERE ***"
  fringe = util.Queue();
  expanded = set();
  fringe.push((problem.getStartState(),[],0));
  
  while not fringe.isEmpty():
        curState, curMoves, curCost = fringe.pop();

        if(curState in expanded or curState in fringe):
            continue;

        expanded.add(curState);

        if problem.isGoalState(curState):
            return curMoves;

        for state, direction, cost in problem.getSuccessors(curState):
            fringe.push((state, curMoves+[direction], curCost));            
  return [];
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  "Search the node that has the lowest combined cost"
  explored = {}
  path_costs = {}
  "FORMAT: (location, action, parent, step_cost)"
  fringe = util.PriorityQueue()
  fringe.push((problem.getStartState(), None , None, 0), 0)
  firstTime = True
    
  while True:
    if fringe.isEmpty():
        return []
        
    node, action, parent, step_cost = fringe.pop()
    if firstTime:
        explored[node] = []
        path_costs[node] = 0
        firstTime = False
    else:
        Path = list(explored[parent])
        Path.append(action)
        explored[node] = Path
        path_costs[node] = path_costs[parent] + step_cost
        
    if problem.isGoalState(node): 
        return explored[node]
        
    successors = problem.getSuccessors(node)
    for successor, action, cost in successors:
        if successor not in explored: #not explored
            "check if in frontier, if it is, check the cost, keep the lower cost"
            isInFrontier = False
            for (index, (priority, test_node)) in enumerate(fringe.heap):
                if successor == test_node[0]:
                    isInFrontier = True
                    if path_costs[node] + cost < path_costs[test_node[2]] + test_node[3]:
                        del fringe.heap[index]
                        fringe.push((successor, action, node, step_cost), path_costs[node] + cost)
                            
            if not isInFrontier:
                fringe.push((successor, action, node, cost), path_costs[node] + cost)


def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  explored = {}
  path_costs = {}
  "FORMAT: (location, action, parent, step_cost)"
  fringe = util.PriorityQueue()
  fringe.push((problem.getStartState(), None , None, 0), 0)
  firstTime = True
    
  while True:
    if fringe.isEmpty():
        return []
        
    node, action, parent, step_cost = fringe.pop()
    "Path  & Path cost recording"
    if firstTime:
        explored[node] = []
        path_costs[node] = 0
        firstTime = False
    else:
        Path = list(explored[parent])
        Path.append(action)
        explored[node] = Path
        path_costs[node] = path_costs[parent] + step_cost
        
    if problem.isGoalState(node): 
        return explored[node]
        
    successors = problem.getSuccessors(node)
    for successor, action, cost in successors:
        if successor not in explored: #not explored
            "check not in frontier, if is, check the cost, keep the lower cost"
            isInFrontier = False
            for (index, (priority, test_node)) in enumerate(fringe.heap):
                if successor == test_node[0]:
                    isInFrontier = True
                    if path_costs[node] + cost < path_costs[test_node[2]] + test_node[3]:
                        del fringe.heap[index]
                        fringe.push((successor, action, node, step_cost), path_costs[node] + cost + heuristic(successor, problem))
                            
            if not isInFrontier:
                fringe.push((successor, action, node, cost), path_costs[node] + cost + heuristic(successor, problem))

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch