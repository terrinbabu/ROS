#!/usr/bin/env python
PACKAGE = 'k2_client_vis'
import numpy
import rospy
import rospkg
from tf import TransformBroadcaster, TransformListener
from k2_client.msg import BodyArray

import logging

logger = logging.getLogger('kinWrapper')
logger.setLevel(logging.INFO)

#SEGWAY_SIM = False
#to be use when the segway is not in simualtion mode (SEGWAY_SIM = False)
REF_FRAME1 = '/head/skel_depth_frame'
#to be use when the segway is in simulation mode (SEGWAY_SIM = True)
#REF_FRAME2 = '/head/skel_depth_frame2'

#REF_HB = '/herb_base'
#REF_MAP = '/map'

JOINTS = {                                #TP to reduce no. of tf
    0: 'SpineBase',
    1: 'SpineMid',
    2: 'Neck',
    3: 'Head',
    4: 'ShoulderLeft',
    5: 'ElbowLeft',
    6: 'WristLeft',
    7: 'HandLeft',
    8: 'ShoulderRight',
    9: 'ElbowRight',
    10: 'WristRight',
    11: 'HandRight',
    12: 'HipLeft',
    13: 'KneeLeft',
    14: 'AnkleLeft',
    15: 'FootLeft',
    16: 'HipRight',
    17: 'KneeRight',
    18: 'AnkleRight',
    19: 'FootRight',
    20: 'SpineShoulder',
    21: 'HandTipLeft',
    22: 'ThumbLeft',
    23: 'HandTipRight',
    24: 'ThumbRight'
}

##JOINTS = {                               
    ###0: 'SpineBase',
    ###1: 'SpineMid',
    ###2: 'Neck',
    ###3: 'Head',
    ###4: 'ShoulderLeft',
    ###5: 'ElbowLeft',
    ###6: 'WristLeft',
    ##7: 'HandLeft',
    ###8: 'ShoulderRight',
    ###9: 'ElbowRight',
    ###10: 'WristRight',
    ##11: 'HandRight',
    ###12: 'HipLeft',
    ###13: 'KneeLeft',
    ###14: 'AnkleLeft',
    ###15: 'FootLeft',
    ###16: 'HipRight',
    ###17: 'KneeRight',
    ###18: 'AnkleRight',
    ###19: 'FootRight',
    ###20: 'SpineShoulder',
    ###21: 'HandTipLeft',
    ###22: 'ThumbLeft',
    ###23: 'HandTipRight',
    ###24: 'ThumbRight'
##}

class kinWrapper:
    def __init__(self, br, pos_sys, quat_sys):
         self.br = br
         self.id = 0
         self.pos_sys = pos_sys
         self.quat_sys = quat_sys
    
    #def getFullTfName(self, tf_name):
        #return '/K2/user/' + tf_name    
      
    def getFullTfNameId(self, tf_name, id):
        return '/K2/user' + str(self.id) + '/' + tf_name
    
    def transform(self, currpos, currorient, tf_name, parent_frame, time, id=''):
        if numpy.sum(currorient) == 0:
            currorient = (0,0,0,1)
        
        try:
            self.br.sendTransform(currpos, 
                            currorient, 
                            time, 
                            self.getFullTfNameId(tf_name,id),
                            parent_frame)
            
        except Exception, e:
            print 'error: ', str(e)
            logger.error('Failure: %s' % str(e))

            

    def kinect_callback(self, ref_systems):
        #now = rospy.get_rostime()
        #print now.secs
        #logger.info("Current time %i %i", now.secs, now.nsecs)        
        #time = rospy.Time.now()   
        time = rospy.get_rostime()
        self.id = 0
        for body in ref_systems.bodies:
            self.id += 1
            if body.isTracked:
                rate = rospy.Rate(200)
                for i in range(25):                            #TP to reduce no. of tf
                #joint_num = [7,11]                             #
                #for i in joint_num:                            #
                    currpos = (body.jointPositions[i].position.x, 
                                body.jointPositions[i].position.y, 
                                body.jointPositions[i].position.z)
                    currorient = (body.jointOrientations[i].orientation.x, 
                                    body.jointOrientations[i].orientation.y, 
                                    body.jointOrientations[i].orientation.z, 
                                    body.jointOrientations[i].orientation.w)
                    
                    #if SEGWAY_SIM == False:   
                    self.transform(currpos, 
                                    currorient, 
                                    JOINTS[i], 
                                    REF_FRAME1, 
                                    time, self.id                                   ) 
                    #else:
                        #try:
                            #self.br.sendTransform(self.pos_sys, 
                                                  #self.quat_sys, 
                                                  #time, 
                                                  #REF_FRAME2,
                                                  #REF_MAP)
                        #except Exception, e:
                                #print 'error: ', str(e)
                                #logger.error('Failure: %s' % str(e))
                       
                        #self.transform(currpos, 
                                            #currorient, 
                                            #JOINTS[i], 
                                            #REF_FRAME2, 
                                            #time) 
                                  
                rate.sleep()


    def kinect_listener(self):  
        rospy.Subscriber('/head/kinect2/bodyArray', BodyArray, self.kinect_callback)
        rospy.spin()
   
if __name__ == '__main__':
    rospy.init_node('from_kin_to_tf') 
    br = TransformBroadcaster()
    tl = TransformListener()       
    #SEGWAY_SIM = rospy.get_param("~seg_sim");   
    rospy.sleep(1.)   
    pos_sys = None   
    quat_sys = None        
    #if SEGWAY_SIM == True:             
        #try:     
            #time = tl.getLatestCommonTime(REF_FRAME1, REF_HB)
            #pos_sys, quat_sys = tl.lookupTransform(REF_HB, REF_FRAME1, time)
        #except:
            #logger.error('Rototranslation matrix between /herb_base and /head/skel_depth_frame not avaialble')
            #print 'Rototranslation matrix between /herb_base and /head/skel_depth_frame not avaialble'
            #exit()
            
        #pos_sys = list(pos_sys)     
        #pos_sys[0] = pos_sys[0] - 0.14       
        #pos_sys[2] = pos_sys[2] + 0.1 #-0.05       
        #pos_sys = tuple(pos_sys)       

    kindata = kinWrapper(br, pos_sys, quat_sys)  
    #while not rospy.is_shutdown():          
    kindata.kinect_listener()
