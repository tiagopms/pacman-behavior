#!/usr/bin/env python

import rospy
import sys
import pacman

def runPacman():
    rospy.init_node('pacman_interface', anonymous=True)

    a=["-p", "RosAgent", "-l", "smallClassic", "-k", "3"]
    args = pacman.readCommand(a)
    pacman.runGames(**args)


if __name__ == '__main__':
    try:
        runPacman()
    except rospy.ROSInterruptException: pass
    
