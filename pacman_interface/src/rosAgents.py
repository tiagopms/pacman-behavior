# keyboardAgents.py
# -----------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

from game import Agent
from game import Directions
import random
import rospy
from pacman_interface.msg import PacmanAction
from pacman_interface.srv import PacmanGetAction


class RosAgent(Agent):
    """
    An agent controlled by ROS messages.
    """
    directions = [Directions.WEST,
                  Directions.EAST,
                  Directions.NORTH,
                  Directions.SOUTH,
                  Directions.STOP]

    def __init__( self, index = 0 ):

        self.nextMove = None
        self.lastMove = Directions.STOP
        self.index = index
        self.keys = []
        
        rospy.Subscriber("/pacman_interface/pacman_action", PacmanAction, self.actionCallback)

    def actionCallback(self, data):
        self.nextMove = None

        if data.action >= 0 and data.action <= 5:
            self.nextMove = self.directions[data.action]

    def getAction(self, state):
       # from graphicsUtils import keys_waiting
       # from graphicsUtils import keys_pressed
       # keys = keys_waiting() + keys_pressed()
       # if keys != []:
       #     self.keys = keys

        legal = state.getLegalActions(self.index)
        move = self.getMove(legal)

        if move == None:
            # Try to move in the same direction as before if impossible, stop
            if self.lastMove in legal:
                move = self.lastMove
            else:
                move = Directions.STOP

        if move not in legal:
            move = random.choice(legal)

        self.lastMove = move
        return move

    def getMove(self, legal):
        move = None

        if self.nextMove in legal:
            move = self.nextMove
        self.nextMove = None

        return move


class RosWaitAgent(Agent):
    """
    An agent controlled by ROS messages.
    """
    directions = [Directions.WEST,
                  Directions.EAST,
                  Directions.NORTH,
                  Directions.SOUTH,
                  Directions.STOP]
    actionToMovement = {PacmanAction.WEST:  Directions.WEST,
                        PacmanAction.EAST:  Directions.EAST,
                        PacmanAction.NORTH: Directions.NORTH,
                        PacmanAction.SOUTH: Directions.SOUTH,
                        PacmanAction.STOP:  Directions.STOP}

    def __init__( self, index = 0 ):

        self.nextMove = None
        self.lastMove = Directions.STOP
        self.index = index
        self.keys = []

    def getAction(self, state):
        legal = state.getLegalActions(self.index)
        move = Directions.STOP
        
        rospy.wait_for_service('/get_action')
        try:
            rosGetAction = rospy.ServiceProxy('get_action', PacmanGetAction)
            servResponse = rosGetAction()
            print servResponse
            move = self.actionToMovement[servResponse.action.action]
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        if move not in legal:
            move = Directions.STOP

        return move

    def fromActionToMove(self, action):
        move = None

        return move
