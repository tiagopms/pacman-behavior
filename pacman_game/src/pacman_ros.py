#!/usr/bin/env python

import rospy
import pacman
from pacman_msgs.srv import StartGame
from pacman_msgs.srv import EndGame


start_game = False
show_gui = False


class PacmanGame():
    """
    A game started and controlled by ROS messages and services.
    """
    def __init__( self, index = 0 ):
        #start ros node
        rospy.init_node('pacman_game', anonymous=True)
        self.r = rospy.Rate(10) # 10hz

        # game attributes
        a=["-p", "RosWaitServiceAgent", "-l", "smallClassic", "-k", "4"]
        self.args = pacman.readCommand(a)

        # service and variables to start new game and end it
        self.start_game_srv = rospy.Service('/pacman/start_game', StartGame, self.start_game_service)
        self.end_game_client = rospy.ServiceProxy('/pacman/end_game', EndGame)
        self.start_game = False
        self.show_gui = False

    def start_game_service(self, req):
        if self.start_game:
            rospy.logwarn("Trying to start already started game")
            return False
        self.start_game = True
        self.show_gui = req.show_gui
        return True

    def end_game(self, is_win):
        # set game as not running
        self.start_game = False
        self.show_gui = False

        # call end game service
        rospy.loginfo("Ending game!")
        rospy.wait_for_service('/pacman/end_game')
        try:
          srv_resp = self.end_game_client(is_win)
          if not srv_resp.game_restarted:
            rospy.signal_shutdown("Shuting down node, game ended and wasn't restarted")
        except rospy.ServiceException as exc:
          print("Service did not process request: " + str(exc))

        return True

    # run game
    def run(self):
        is_win = False
        while not rospy.is_shutdown():
            if self.start_game:

                #if not show gui, set game as in training mode
                if not self.show_gui:
                    self.args['numTraining'] = 1

                # run game and get if win or lose
                self.args['pacman'].startEpisode()
                games = pacman.runGames(**self.args)
                single_game = games.pop()
                is_win = single_game.state.isWin()

                # end game
                self.end_game(is_win)

            #sleep while in loop
            self.r.sleep()

if __name__ == '__main__':
    game = PacmanGame()
    try:
        game.run()
    except rospy.ROSInterruptException: pass
    
