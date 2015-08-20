#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import time
from std_msgs.msg import Float64MultiArray

# define state Emergency
class Emergency(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome'])

    def execute(self, userdata):
        rospy.loginfo('In state: EmergencyState')
        #time.sleep(1)
        return 'outcome'

# define state HumanDanger 0
class HumanDanger0(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome'])

    def execute(self, userdata):
    	#rospy.loginfo('In state: HumanDanger0')
    	time.sleep(1)
        return 'outcome'

# define state HumanDanger 1
class HumanDanger1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome'])

    def execute(self, userdata):
        #rospy.loginfo('In state: HumanDanger1')
        time.sleep(1)
        return 'outcome'

# define state HumanDanger 2
class HumanDanger2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome'])

    def execute(self, userdata):
        #rospy.loginfo('In state: HumanDanger2')
        time.sleep(1)
        return 'outcome'

# define state HumanDanger 3
class HumanDanger3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome'])

    def execute(self, userdata):
        #rospy.loginfo('In state: HumanDanger3')
        time.sleep(1)
        return 'outcome'

# define state HumanDanger 4
class HumanDanger4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['tEmergency'])

    def execute(self, userdata):
        #rospy.loginfo('In state: HumanDanger4. To Emergency')
        time.sleep(1)
        return 'tEmergency'

# define state Awareness  (estimate the level of human danger!)
class HumanAwareness(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['tHD0','tHD1','tHD2','tHD3','tHD4','tEmergency'])
        self.humanDanger = 0
        rospy.loginfo('InitState')


    def execute(self, userdata):
        rospy.loginfo('In state: Awareness, estimating awareness...')
        print("Inforamation:  smach.humanDanger2")
        #time.sleep(1)
        self.humanDanger += 1
        if self.humanDanger == 0:
            #rospy.loginfo('Transition to human danger level is: 0')
            return 'tHD0'
        elif self.humanDanger == 1:
            #rospy.loginfo('Transition to human danger level is: 1')
            return 'tHD1'
        elif self.humanDanger == 2:
            #rospy.loginfo('Transition to human danger level is: 2')
            return 'tHD2'
        elif self.humanDanger == 3:
            #rospy.loginfo('Transition to human danger level is: 3')
            return 'tHD3'
        elif self.humanDanger == 4:
            #rospy.loginfo('Transition to human danger level is: 4')
            return 'tHD4'
        else:
            rospy.loginfo('Invalid state: Go to emergency.')
            return 'tEmergency'


#class HumanAwareness:
    #def __init__(self):

def callbackPedDetected(data):
    rospy.loginfo("DetectionReceived")
    print data.data

# main
def main():
    rospy.init_node('human_awareness', anonymous=True)
    #rospy.init_node('kalman_filter', anonymous=True)
    

    rospy.Subscriber("/ped/DistanceAngle", Float64MultiArray, callbackPedDetected)

    # Create a SMACH state machine
    sm_safety = smach.StateMachine(outcomes=['HumanDanger0','HumanDanger1','HumanDanger2','HumanDanger3','Emergency'])
    smach.humanDanger2 = 123;

    

    # Open the container
    with sm_safety:
        # Add states to the container        
        smach.StateMachine.add('HUMANAWARENESS', HumanAwareness(), transitions={'tHD0':'HUMANDANGER0', 'tHD1':'HUMANDANGER1', 'tHD2':'HUMANDANGER2', 'tHD3':'HUMANDANGER3', 'tHD4':'HUMANDANGER4', 'tEmergency':'EMERGENCY'})
        smach.StateMachine.add('EMERGENCY', Emergency(), transitions={'outcome':'Emergency'}) 
        smach.StateMachine.add('HUMANDANGER0', HumanDanger0(), transitions={'outcome':'HumanDanger0'})
        smach.StateMachine.add('HUMANDANGER1', HumanDanger1(), transitions={'outcome':'HumanDanger1'})
        smach.StateMachine.add('HUMANDANGER2', HumanDanger2(), transitions={'outcome':'HumanDanger2'})
        smach.StateMachine.add('HUMANDANGER3', HumanDanger3(), transitions={'outcome':'HumanDanger3'})
        smach.StateMachine.add('HUMANDANGER4', HumanDanger4(), transitions={'tEmergency':'EMERGENCY'})

    # Create a Introspection Server for visualization ($rosrun smach_viewer smach_viewer.py)
    sis = smach_ros.IntrospectionServer('server_name',sm_safety,'/SM_ROOT')
    sis.start()
    
    rospy.loginfo('BeforeExecution')
    
    # Execute SMACH plan
    outcome = sm_safety.execute()
    rospy.loginfo('AfterExecution')

    rospy.spin()



if __name__ == '__main__':
    main()
