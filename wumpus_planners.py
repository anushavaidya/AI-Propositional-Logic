# wumpus_planners.py
# ------------------
# Licensing Information:
# Please DO NOT DISTRIBUTE OR PUBLISH solutions to this project.
# You are free to use and extend these projects for EDUCATIONAL PURPOSES ONLY.
# The Hunt The Wumpus AI project was developed at University of Arizona
# by Clay Morrison (clayton@sista.arizona.edu), spring 2013.
# This project extends the python code provided by Peter Norvig as part of
# the Artificial Intelligence: A Modern Approach (AIMA) book example code;
# see http://aima.cs.berkeley.edu/code.html
# In particular, the following files come directly from the AIMA python
# code: ['agents.py', 'logic.py', 'search.py', 'utils.py']
# ('logic.py' has been modified by Clay Morrison in locations with the
# comment 'CTM')
# The file ['minisat.py'] implements a slim system call wrapper to the minisat
# (see http://minisat.se) SAT solver, and is directly based on the satispy
# python project, see https://github.com/netom/satispy .

from wumpus_environment import *
from wumpus_kb import *
import search

#-------------------------------------------------------------------------------
# Distance fn
#-------------------------------------------------------------------------------

def manhattan_distance_with_heading(current, target):
    """
    Return the Manhattan distance + any turn moves needed
        to put target ahead of current heading
    current: (x,y,h) tuple, so: [0]=x, [1]=y, [2]=h=heading)
    heading: 0:^:north 1:<:west 2:v:south 3:>:east
    """
    md = abs(current[0] - target[0]) + abs(current[1] - target[1])
    if current[2] == 0:   # heading north
        # Since the agent is facing north, "side" here means
        # whether the target is in a row above or below (or
        # the same) as the agent.
        # (Same idea is used if agent is heading south)
        side = (current[1] - target[1])
        if side > 0:
            md += 2           # target is behind: need to turns to turn around
        elif side <= 0 and current[0] != target[0]:
            md += 1           # target is ahead but not directly: just need to turn once
        # note: if target straight ahead (curr.x == tar.x), no turning required
    elif current[2] == 1: # heading west
        # Now the agent is heading west, so "side" means
        # whether the target is in a column to the left or right
        # (or the same) as the agent.
        # (Same idea is used if agent is heading east)
        side = (current[0] - target[0])
        if side < 0:
            md += 2           # target is behind
        elif side >= 0 and current[1] != target[1]:
            md += 1           # target is ahead but not directly
    elif current[2] == 2: # heading south
        side = (current[1] - target[1])
        if side < 0:
            md += 2           # target is behind
        elif side >= 0 and current[0] != target[0]:
            md += 1           # target is ahead but not directly
    elif current[2] == 3: # heading east
        side = (current[0] - target[0])
        if side > 0:
            md += 2           # target is behind
        elif side <= 0 and current[1] != target[1]:
            md += 1           # target is ahead but not directly
    return md


#-------------------------------------------------------------------------------
# Plan Route
#-------------------------------------------------------------------------------

def plan_route(current, heading, goals, allowed):
    """
    Given:
       current location: tuple (x,y)
       heading: integer representing direction
       gaals: list of one or more tuple goal-states
       allowed: list of locations that can be moved to
    ... return a list of actions (no time stamps!) that when executed
    will take the agent from the current location to one of (the closest)
    goal locations
    You will need to:
    (1) Construct a PlanRouteProblem that extends search.Problem
    (2) Pass the PlanRouteProblem as the argument to astar_search
        (search.astar_search(Problem)) to find the action sequence.
        Astar returns a node.  You can call node.solution() to exract
        the list of actions.
    NOTE: represent a state as a triple: (x, y, heading)
          where heading will be an integer, as follows:
          0='north', 1='west', 2='south', 3='east'
    """

    # Ensure heading is a in integer form
    if isinstance(heading,str):
        heading = Explorer.heading_str_to_num[heading]

    if goals and allowed:
        prp = PlanRouteProblem((current[0], current[1], heading), goals, allowed)
        # NOTE: PlanRouteProblem will include a method h() that computes
        #       the heuristic, so no need to provide here to astar_search()
        node = search.astar_search(prp)
        if node:
            return node.solution()
    
    # no route can be found, return empty list
    return []

#-------------------------------------------------------------------------------

class PlanRouteProblem(search.Problem):
    def __init__(self, initial, goals, allowed):
        """ Problem defining planning of route to closest goal
        Goal is generally a location (x,y) tuple, but state will be (x,y,heading) tuple
        initial = initial location, (x,y) tuple
        goals   = list of goal (x,y) tuples
        allowed = list of state (x,y) tuples that agent could move to """
        self.initial = initial # initial state
        self.goals = goals     # list of goals that can be achieved
        self.allowed = allowed # the states we can move into

    def h(self,node):
        """
        Heuristic that will be used by search.astar_search()
        """
        "*** YOUR CODE HERE ***"  
       
        goal_distance = []
        for goal in self.goals:
            dist = manhattan_distance_with_heading(node.state,goal)
            goal_distance.append(dist)
        return min(goal_distance)
        

    def actions(self, state):
        """
        Return list of allowed actions that can be made in state
        """
        "*** YOUR CODE HERE ***"
        #actions allowed if the agent doesnt encounter a wall, wumpus or a pit
        without_wall_pit_wumpus = ['Forward','TurnRight','TurnLeft']
        #actions allowed if the agent encounters a wall wumpus or a pit
        with_wall_pit_wumpus = ['TurnRight','TurnLeft']
        x = state[0] 
        y = state[1]
        direction = state[2]
        North = 0
        West = 1
        South = 2
        East = 3
        
        if direction == North and (x,y+1) in self.allowed:
            return without_wall_pit_wumpus
        elif direction == West and (x-1,y) in self.allowed:
            return without_wall_pit_wumpus
        elif direction == South and (x,y-1) in self.allowed:
            return without_wall_pit_wumpus
        elif direction == East and (x+1,y) in self.allowed:
            return without_wall_pit_wumpus
        return with_wall_pit_wumpus
        


    def result(self, state, action):
        """
        Return the new state after applying action to state
        """
        "*** YOUR CODE HERE ***"
        x = state[0]
        y = state[1]
        direction = state[2]
        North = 0
        West = 1
        South = 2
        East = 3
        if action == 'TurnRight':
           
            if direction == North:
                return (x,y, East)
            elif direction == West:
                return (x,y, North)
            elif direction == South:
                return(x,y,West)
            elif direction == East:
                return (x,y, South)

        if action == 'TurnLeft':
            if direction == North:
                return (x,y,West)
            elif direction == West:
                return (x,y,South)
            elif direction == South:
                return (x,y,East)
            elif direction == East:
                return (x,y,North)

        if action == 'Forward':
            if direction == North:
                return (x,y + 1,North)
            elif direction == West:
                return (x-1,y,West)
            elif direction == South:
                return (x,y-1,South)
            elif direction == East:
                return (x+1,y,East)
        

    def goal_test(self, state):
        """
        Return True if state is a goal state
        """
        "*** YOUR CODE HERE ***"
        return state[0:2] in self.goals

#-------------------------------------------------------------------------------

def test_PRP(initial):
    """
    The 'expected initial states and solution pairs' below are provided
    as a sanity check, showing what the PlanRouteProblem soluton is
    expected to produce.  Provide the 'initial state' tuple as the
    argument to test_PRP, and the associate solution list of actions is
    expected as the result.
    The test assumes the goals are [(2,3),(3,2)], that the heuristic fn
    defined in PlanRouteProblem uses the manhattan_distance_with_heading()
    fn above, and the allowed locations are:
        [(0,0),(0,1),(0,2),(0,3),
        (1,0),(1,1),(1,2),(1,3),
        (2,0),            (2,3),
        (3,0),(3,1),(3,2),(3,3)]
    
    Expected intial state and solution pairs:
    (0,0,0) : ['Forward', 'Forward', 'Forward', 'TurnRight', 'Forward', 'Forward']
    (0,0,1) : ['TurnRight', 'Forward', 'Forward', 'Forward', 'TurnRight', 'Forward', 'Forward']
    (0,0,2) : ['TurnLeft', 'Forward', 'Forward', 'Forward', 'TurnLeft', 'Forward', 'Forward']
    (0,0,3) : ['Forward', 'Forward', 'Forward', 'TurnLeft', 'Forward', 'Forward']
    """
    return plan_route((initial[0],initial[1]), initial[2],
                      # Goals:
                      [(2,3),(3,2)],
                      # Allowed locations:
                      [(0,0),(0,1),(0,2),(0,3),
                       (1,0),(1,1),(1,2),(1,3),
                       (2,0),            (2,3),
                       (3,0),(3,1),(3,2),(3,3)])


#-------------------------------------------------------------------------------
# Plan Shot
#-------------------------------------------------------------------------------

def plan_shot(current, heading, goals, allowed):
    """ Plan route to nearest location with heading directed toward one of the
    possible wumpus locations (in goals), then append shoot action.
    NOTE: This assumes you can shoot through walls!!  That's ok for now. """
    if goals and allowed:
        psp = PlanShotProblem((current[0], current[1], heading), goals, allowed)
        node = search.astar_search(psp)
        if node:
            plan = node.solution()
            plan.append(action_shoot_str(None))
            # HACK:
            # since the wumpus_alive axiom asserts that a wumpus is no longer alive
            # when on the previous round we perceived a scream, we
            # need to enforce waiting so that itme elapses and knowledge of
            # "dead wumpus" can then be inferred...
            plan.append(action_wait_str(None))
            return plan

    # no route can be found, return empty list
    return []

#-------------------------------------------------------------------------------

class PlanShotProblem(search.Problem):
    def __init__(self, initial, goals, allowed):
        """ Problem defining planning to move to location to be ready to
              shoot at nearest wumpus location
        NOTE: Just like PlanRouteProblem, except goal is to plan path to
              nearest location with heading in direction of a possible
              wumpus location;
              Shoot and Wait actions is appended to this search solution
        Goal is generally a location (x,y) tuple, but state will be (x,y,heading) tuple
        initial = initial location, (x,y) tuple
        goals   = list of goal (x,y) tuples
        allowed = list of state (x,y) tuples that agent could move to """
        self.initial = initial # initial state
        self.goals = goals     # list of goals that can be achieved
        self.allowed = allowed # the states we can move into

    def h(self,node):
        """
        Heuristic that will be used by search.astar_search()
        """
        "*** YOUR CODE HERE ***"
        wumpus_states = self.goals
        allowed_states = self.allowed
        shoot_states = []
        distance = []

        for position in wumpus_states:
            w_x = position[0]
            w_y = position[1]
            for each_state in allowed_states:
                a_x = each_state[0]
                a_y = each_state[1]
                if (w_x == a_x) or (w_y == a_y):
                    shoot_states.append(each_state)

        
        for location in shoot_states:
            man_dist = manhattan_distance_with_heading(node.state, location)
            distance.append(man_dist)
        return min(distance)
        
        

    def actions(self, state):
        """
        Return list of allowed actions that can be made in state
        """
        "*** YOUR CODE HERE ***"
        #actions allowed if the agent doesnt encounter a wall, wumpus or a pit
        without_wall_pit_wumpus = ['Forward','TurnRight','TurnLeft']
        #actions allowed if the agent encounters a wall wumpus or a pit
        with_wall_pit_wumpus = ['TurnRight','TurnLeft']
        x = state[0] 
        y = state[1]
        heading = state[2]
        North = 0
        West = 1
        South = 2
        East = 3
        
        if heading == North and (x, y + 1) in self.allowed:
            return without_wall_pit_wumpus
        elif heading == West and (x - 1, y) in self.allowed:
            return without_wall_pit_wumpus
        elif heading == South and (x, y - 1) in self.allowed:
            return without_wall_pit_wumpus
        elif heading == East and (x + 1, y) in self.allowed:
            return without_wall_pit_wumpus
        return with_wall_pit_wumpus
        

    def result(self, state, action):
        """
        Return the new state after applying action to state
        """
        "*** YOUR CODE HERE ***"
        x = state[0]
        y = state[1]
        heading = state[2]
        North = 0
        West = 1
        South = 2
        East = 3
        if action == 'TurnRight':
            if heading == North:
                return (x,y,East)
            elif heading == West:
                return (x,y,North)
            elif heading == South:
                return(x,y,West)
            elif heading == East:
                return (x,y,East)

        if action == 'TurnLeft':
            if heading == North:
                return (x,y,West)
            elif heading == West:
                return (x,y,South)
            elif heading == South:
                return (x,y,East)
            elif heading == East:
                return (x,y,North)

        if action == 'Forward':
            if heading == North:
                return (x,y + 1,North)
            elif heading == West:
                return (x-1,y,West)
            elif heading == South:
                return (x,y-1,South)
            elif heading == East:
                return (x+1,y,East)



        

    def goal_test(self, state):
        """
        Return True if state is a goal state
        """
        "*** YOUR CODE HERE ***"
       
        
        wumpus_states = self.goals
        x = state[0]
        y = state[1]
        heading = state[2]
        North = 0
        West = 1
        South = 2
        East = 3
        if state in self.goals:
            return False

        for position in wumpus_states:
            w_x= position[0]
            w_y = position[1]
            if w_x == x:
                if ((w_y > y) and heading == North):
                    return True
                if ((w_y < y) and heading == South):
                    return True
            if w_y == y:
                if ((w_x < x) and heading == West):
                    return True
                if ((w_x > x) and heading == East):
                    return True
        return False

#-------------------------------------------------------------------------------

def test_PSP(initial = (0,0,3)):
    """
    The 'expected initial states and solution pairs' below are provided
    as a sanity check, showing what the PlanShotProblem soluton is
    expected to produce.  Provide the 'initial state' tuple as the
    argumetn to test_PRP, and the associate solution list of actions is
    expected as the result.
    The test assumes the goals are [(2,3),(3,2)], that the heuristic fn
    defined in PlanShotProblem uses the manhattan_distance_with_heading()
    fn above, and the allowed locations are:
        [(0,0),(0,1),(0,2),(0,3),
        (1,0),(1,1),(1,2),(1,3),
        (2,0),            (2,3),
        (3,0),(3,1),(3,2),(3,3)]
    
    Expected intial state and solution pairs:
    (0,0,0) : ['Forward', 'Forward', 'TurnRight', 'Shoot', 'Wait']
    (0,0,1) : ['TurnRight', 'Forward', 'Forward', 'TurnRight', 'Shoot', 'Wait']
    (0,0,2) : ['TurnLeft', 'Forward', 'Forward', 'Forward', 'TurnLeft', 'Shoot', 'Wait']
    (0,0,3) : ['Forward', 'Forward', 'Forward', 'TurnLeft', 'Shoot', 'Wait']
    """
    return plan_shot((initial[0],initial[1]), initial[2],
                     # Goals:
                     [(2,3),(3,2)],
                     # Allowed locations:
                     [(0,0),(0,1),(0,2),(0,3),
                      (1,0),(1,1),(1,2),(1,3),
                      (2,0),            (2,3),
                      (3,0),(3,1),(3,2),(3,3)])
    
#-------------------------------------------------------------------------------
