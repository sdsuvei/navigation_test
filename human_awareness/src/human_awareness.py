#!/usr/bin/env python

import rospy
import time
from collections import namedtuple
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import UInt16

distanceBoundary1 = 1
distanceBoundary2 = 3
distanceBoundary3 = 5
timeBetweenEvaluation = 0.2 # Time between evaulations in seconds
time2ForgetHumans = 5.0 # Forget humans after X seconds
Human = namedtuple('Human', ['distance', 'angle','timeSinceDetection'])

pubHumanDangerLevel = rospy.Publisher("/HumanDangerLevel", UInt16 , queue_size=10)

human = []

def EvaluateHumanAwareness(data):
    global human
    global pubHumanDangerLevel
    msgHumanDangerLevel = UInt16;
    minDistanceToHuman = []
    #print "timer interrupt"
    # Calculate stuff if humans detected.
    if len(human)>0:
        humanNew = []
        # Remove pedestrins detected for more than 
        for n in range(len(human)):
            #print human[n]
            if(human[n].timeSinceDetection>0):
                humanNew.append(Human(human[n].distance, human[n].angle,human[n].timeSinceDetection-timeBetweenEvaluation))

        # Find minimum distnace
        distance = []
        for n in range(len(humanNew)):
            distance.append(humanNew[n].distance)

        if len(distance)>0:
            minDistanceToHuman = min(distance)
            #print minDistanceToHuman
        human = humanNew
    #print minDistanceToHuman, distanceBoundary3, len(human)
    if len(human)==0:
        msgHumanDangerLevel.data = 0
        #print "No human detected."
    elif(minDistanceToHuman<distanceBoundary2) and (minDistanceToHuman>=distanceBoundary1):
        msgHumanDangerLevel.data = 3
        #print "HumanDanger3: Stop and send warning."
    elif (minDistanceToHuman<distanceBoundary3) and (minDistanceToHuman>=distanceBoundary2):
        msgHumanDangerLevel.data = 2
        #print "HumanDanger2: Slow down and send warning."
    elif minDistanceToHuman>=distanceBoundary3:
        msgHumanDangerLevel.data = 1
        #print "HumanDanger1: Human detected far away do nothing..."
    else:
        msgHumanDangerLevel.data = 4
        #print "HumanDanger4: Emergency."
    #print "HumanDangerLevel: ", msgHumanDangerLevel.data
    pubHumanDangerLevel.publish(msgHumanDangerLevel)

def callbackPedDetected(data):
    #rospy.loginfo("DetectionReceived")
    distance = []
    angles = []

    nHumansDetected = len(data.data)/2
    for n in range(nHumansDetected):
        distance = data.data[n*2];
        if distance < 0:
            distance = 10000
        human.append(Human(distance,data.data[n*2+1],time2ForgetHumans))

    #minDistanceToHuman = []
    #if len(distance)>0:
    #    minDistanceToHuman = min(distance)
    # print angles
    # print distance
    # print minDistanceToHuman





# main
def main():
    rospy.init_node('human_awareness', anonymous=True)

    rospy.Subscriber("/ped/DistanceAngle", Float64MultiArray, callbackPedDetected)
    rospy.Timer(rospy.Duration(timeBetweenEvaluation), EvaluateHumanAwareness)
    rospy.spin()


if __name__ == '__main__':
    main()
