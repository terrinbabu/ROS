PACKAGE = 'hrc'
import numpy
import rospy
import rospkg
import logging
import copy
import collections
import sys, subprocess, os, signal
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Float32MultiArray, Bool
from msg import PoseArrays
from scipy import signal
from tf import transformations, TransformListener
import herbpy
from prpy.planning import Sequence, VectorFieldPlanner, TSRPlanner
from prpy.base import wam
from prpy import util, viz
from prpy.planning.exceptions import CollisionPlanningError, SelfCollisionPlanningError
import random
import openravepy
import roslaunch


BASE_FRAME = '/map'
REF_OBJ = ['glass1','glass2','glass3','box1','box2','box3', 'box4',
           'box5','box6','box7','box8','box9']

logger = logging.getLogger('assistance')
logger.setLevel(logging.INFO)


class Assistance(object):
    def __init__(self, id, env, continuous=True, herb_sim=True, segway_sim=True, action='stamp'):
        assert id != ''
        self.id = id
        self.env = env
        self.robot = env.GetRobot('herb')
        self.herb_sim = herb_sim
        self.action = action
        self.envlock = False
        self.closing = False
        self.continuous = continuous
        
        self.over_hand = False
        self.over_hand_value = Pose()  
        self.homeconf = numpy.array([ 3.70017679, -1.475547  ,  0.55416338,  2.0397097 , -0.32601491,
                                    -0.36717559,  1.53363693])
             
        self.nottouchedobj = []
        self.tsr_box_distance = 0.12 #0.14
        if continuous==True:
            self.filter_options = openravepy.IkFilterOptions.CheckEnvCollisions  #or 0 for no collision checks            
            self.tsrviz = []   # needed in order to visualize tsr continuosly
            self.tsrs = []
            self.tsrmsg = self.tsrchain()
            self.min_tsr = numpy.identity(4)      
        self.tf = TransformListener()         
        self.restartnode = False            
        
        cubeexist = False
        for i in env.GetBodies():
            if i.GetName == 'cube' + str(id):
                cubeexist = True
                break
        if cubeexist == False:
            self.cube = openravepy.RaveCreateKinBody(env,'')
            self.cube.SetName('cube' + str(id))
            self.cube.InitFromBoxes(numpy.array([[0,0,0,0.01,0.01,0.01]]),True) # set geometry as one box of extents 0.1, 0.2, 0.3
            self.cube.Enable(False)
            env.AddKinBody(self.cube)  
            
        self.node_rh = rospy.Publisher(self.getFullTfName('rhand_pos'), Pose, queue_size=10)
        self.node_lh = rospy.Publisher(self.getFullTfName('lhand_pos'), Pose, queue_size=10)
        self.env_obj_pose = rospy.Publisher('/env_obj/pos', PoseArray, queue_size=10)       
        self.robot_eep = rospy.Publisher('/herb/active_eep', Pose, queue_size=10)
        self.restart = rospy.Publisher('/restart', Bool, queue_size=10)
        self.prob_goal = rospy.Subscriber('/skel/prob_goal_' + self.id, Float32MultiArray, 
                                            self.callback_color_obj, queue_size=1)
        self.node_rh_bool_sub = rospy.Subscriber('/skel/hand_override', Bool,
                                                self.callback_override_hand, queue_size=10)
        self.node_rh_sub = rospy.Subscriber('/skel/hand', Pose,
                                                self.callback_new_hand, queue_size=10)

        if continuous==True:
            self.env_obj_tsr = rospy.Publisher('/env_obj/tsr', PoseArrays, queue_size=10)
            self.mint_sr = rospy.Subscriber('/herb/mintsr', Float32MultiArray,
                                            self.callback_update_min_tsr, queue_size=10)           
            self.rob_up_twist = rospy.Subscriber('/herb/next_eep', Float32MultiArray,
                                            self.callback_update_twist, queue_size=10)

        import re                                      
        id_num = [int(s) for s in re.findall('\\d+', self.id)]
        rospy.set_param("/global_user", str(id_num[0]))
        self.rate = rospy.Rate(150) # 10hz
        rospy.sleep(10)

    def getFullTfName(self, link_name):
        return '/skel/' + self.id + '/' + link_name
    
    def define_pose_from_eetransform(self, ee_pose):
        ee_quat = transformations.quaternion_from_matrix(ee_pose)        
        pose = Pose()
        pose.position.x = ee_pose[0,3]
        pose.position.y = ee_pose[1,3]
        pose.position.z = ee_pose[2,3]
        pose.orientation.x = ee_quat[0]
        pose.orientation.y = ee_quat[1]
        pose.orientation.z = ee_quat[2]
        pose.orientation.w = ee_quat[3]        
        return pose   
    
    def define_pose_from_posquat(self, pos, quat):
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]        
        return pose  
       


    def shutdown(self):
        self.robot.PlanToNamedConfiguration('home', execute=True)
        rospy.sleep(5)
        raw_input('---press enter to close---')
        self.closing = True 
        rospy.sleep(1)
        rospy.signal_shutdown('Task done!!!!')

    def tsrchain(self):
        tsrmsg = PoseArrays()
        tsrnb =  10 
        tsrnb_maxtrials = 20000
        self.tsrs = []
        
        if len(self.nottouchedobj) == 0:
            init_nottouchedobj = True
        else:
            init_nottouchedobj = False

        count = -1
        for obj in self.env.GetBodies():
            for ref_name in REF_OBJ:
                if ref_name in obj.GetName():
                    count += 1
                    singletsr = []
                    
                    
                    tsr_list = self.robot.tsrlibrary(obj, self.action)
                  
                    if init_nottouchedobj:
                        self.nottouchedobj.append(True)
                    
                    if all(x==False for x in self.nottouchedobj) == True:
                        self.shutdown()
                    
                    posar = PoseArray()
                    if self.nottouchedobj[count] == True:
                        tsr_chain_idx = random.randint(0, len(tsr_list) - 1)                  
                        tsr_chain = tsr_list[tsr_chain_idx]
                        idx = 0
                        idt = 0
                        while idx<=tsrnb and idt <= tsrnb_maxtrials:
                            #print obj.GetName()
                            idt += 1
                            sample = tsr_chain.sample() 
                            config_tsr = None
                            config_stamp = None
                            rospy.sleep(0.0005)
                            with self.env:
                                config_tsr = self.robot.GetActiveManipulator().FindIKSolution(sample, self.filter_options) # will return None if no config can be found
                                sample_stamp = copy.deepcopy(sample)
                                sample_stamp[2,3] -= self.tsr_box_distance
                                config_stamp = self.robot.GetActiveManipulator().FindIKSolution(sample_stamp, self.filter_options)
                                
                                obj_x = False
                                obj_y = False
                                if config_tsr is not None and config_stamp is not None:
                                    finpos_ee = numpy.array([0.0, 0.0, 0.2, 1.0])
                                    finpos_map = numpy.dot(sample, numpy.transpose(finpos_ee))
                                    obj_aabb = obj.ComputeAABB()
                                    obj_pose = obj.GetTransform()
                                    obj_tol_x = 0.000
                                    obj_tol_y = 0.01

                                    #if finpos_map[0] > obj_pose[0,3] - obj_aabb.extents()[0]/2 + obj_tol_x and finpos_map[0] < obj_pose[0,3] + obj_aabb.extents()[0]/2 - obj_tol_x:                                        
                                        #obj_x = True
                                    obj_x = True
                                    if finpos_map[1] > obj_pose[1,3] - obj_aabb.extents()[1]/2 + obj_tol_y and finpos_map[1] < obj_pose[1,3] + obj_aabb.extents()[1]/2 - obj_tol_y:
                                        obj_y = True   
                            #TODO: keep tsr which allow herb elbow up (reduce the probability to collide with the table)    
                            #prova = self.robot.GetActiveManipulator().FindIKSolutions(sample, filter_options)  
                            #prova2 = self.robot.GetActiveManipulator().FindIKSolutions(sample_stamp, filter_options)
                            
                            if config_tsr is not None and config_stamp is not None and obj_x is True and obj_y is True: 
                                idx += 1
                                singletsr.append(sample)
                                self.tsrviz.append(openravepy.misc.DrawAxes(self.env, sample, dist=0.15))
                                pose = openravepy.poseFromMatrix(sample)
                                quat, xyz = pose[0:4], pose[4:7]
                                tsrpose = Pose()
                                tsrpose.position.x = xyz[0]
                                tsrpose.position.y = xyz[1]
                                tsrpose.position.z = xyz[2]
                                tsrpose.orientation.w = quat[0]
                                tsrpose.orientation.x = quat[1]
                                tsrpose.orientation.y = quat[2]
                                tsrpose.orientation.z = quat[3]
                                posar.poses.append(tsrpose)
                        if len(posar.poses) == 0:  #if the obj is not reachable (not found tsr) a fake tsr is send (tsr so far that it will be discarded by the POMPD)
                            rospy.signal_shutdown('Goal unreachable')
                            self.nottouchedobj[count] = False  #chnage the state of the obj to touched since it cannot be reached
                            tsrpose = Pose()
                            tsrpose.position.x = -9999
                            tsrpose.position.y = -9999
                            tsrpose.position.z = -9999
                            tsrpose.orientation.w = 1
                            tsrpose.orientation.x = 0
                            tsrpose.orientation.y = 0
                            tsrpose.orientation.z = 0
                            posar.poses.append(tsrpose)  
                            singletsr.append(numpy.array([[1.,0.,0.,-9999],
                                                          [0.,1.,0.,-9999],
                                                          [0.,0.,1.,-9999],
                                                          [0.,0.,0.,1.]]))
                    else:
                        tsrpose = Pose()
                        tsrpose.position.x = -9999
                        tsrpose.position.y = -9999
                        tsrpose.position.z = -9999
                        tsrpose.orientation.w = 1
                        tsrpose.orientation.x = 0
                        tsrpose.orientation.y = 0
                        tsrpose.orientation.z = 0
                        posar.poses.append(tsrpose)   
                        singletsr.append(numpy.array([[1.,0.,0.,-9999],
                                                      [0.,1.,0.,-9999],
                                                      [0.,0.,1.,-9999],
                                                      [0.,0.,0.,1.]]))
                    tsrmsg.poses.append(posar)   
                    self.tsrs.append(singletsr)
                    break         
        return tsrmsg

    def publish_poses(self): 
        if self.over_hand==False:            
            tf_name_full = self.getFullTfName('HandTipRight')        
            if self.tf.frameExists(tf_name_full) and self.tf.frameExists(BASE_FRAME):   
                try:                
                    time = self.tf.getLatestCommonTime(tf_name_full, BASE_FRAME)
                    pos, quat = self.tf.lookupTransform(BASE_FRAME, tf_name_full, time)        
                    hand_pose_r = self.define_pose_from_posquat(pos, quat)  
                    self.node_rh.publish(hand_pose_r)
                except:
                    pass
                    
            tf_name_full = self.getFullTfName('HandTipLeft')
            if self.tf.frameExists(tf_name_full) and self.tf.frameExists(BASE_FRAME):   
                try:                
                    time = self.tf.getLatestCommonTime(tf_name_full, BASE_FRAME)
                    pos, quat = self.tf.lookupTransform(BASE_FRAME, tf_name_full, time)        
                    left_pose_r = self.define_pose_from_posquat(pos, quat)    
                    self.node_lh.publish(left_pose_r)    
                except:
                    pass
        else:   
            self.node_rh.publish(self.over_hand_value)            
            self.node_lh.publish(self.over_hand_value)    

    def publish_object_pos(self):
        poselist = PoseArray() 
        for obj in self.env.GetBodies():
            for ref_name in REF_OBJ:                
                if ref_name in obj.GetName():
                    ee_body = obj.GetTransform()                    
                    block_pose = self.define_pose_from_eetransform(ee_body)
                    poselist.poses.append(block_pose)
                    break
        self.env_obj_pose.publish(poselist)
    
    def publish_object_tsr(self):
        self.env_obj_tsr.publish(self.tsrmsg)   #so that I can publish always same tsr poses

       
    def publish_robot_eep(self):  
        robot_pose_ee = self.define_pose_from_eetransform(self.robot.GetActiveManipulator().GetEndEffectorTransform());       
        self.robot_eep.publish(robot_pose_ee)

    def robotplanforprob(self, prob_dist):
        if self.continuous==True:  
            return 0   

        prob = copy.deepcopy(prob_dist)
        index_max = prob.index(max(prob))
        check_prob = False
        for i in range(len(prob_dist)):
            if prob_dist[i] >= 0.5:
                check_prob = True
                break
        if check_prob == False:
            return
                

        if len(self.nottouchedobj) == 0:
            init_nottouchedobj = True
        else:
            init_nottouchedobj = False            

        poselist = []
        for obj in self.env.GetBodies():
            for ref_name in REF_OBJ:                
                if ref_name in obj.GetName(): 
                    #namelist.append(obj.GetName())
                    poselist.append(obj.GetTransform())
                    if init_nottouchedobj:
                        self.nottouchedobj.append(True)

        print self.nottouchedobj 
        if all(x==False for x in self.nottouchedobj) == True:
            self.shutdown()

        #obj closest to the robot different from the human box
        distarray = []
        for i in range(len(poselist)):
            if i is not index_max and self.nottouchedobj[i]==True:                        
                error = poselist[i][0:3,3] - poselist[index_max][0:3,3]
                distarray.append(numpy.dot(numpy.transpose(error), error))
            else:
                distarray.append(100000.0)
        
        print 'prob', prob
        print 'index_max', index_max
        print 'distarray', distarray
        
        if all(x==100000.0 for x in distarray) == True:
            return 0
        
        index_mindist = distarray.index(min(distarray))
        print distarray
        print index_mindist
        count = -1
        for obj in self.env.GetBodies():
            for ref_name in REF_OBJ:                
                if ref_name in obj.GetName(): 
                    count += 1
                    if (count == index_mindist):
                        if self.action == 'stamp':
                            with self.env:
                                tsr_list = self.robot.tsrlibrary(obj, self.action)

                            with viz.RenderTSRList(tsr_list, self.env, render=True):
                                self.robot.GetActiveManipulator().PlanToTSR(tsr_list, num_attempts=20, execute=True)

                            self.planstamp()
                            self.nottouchedobj[count] = False
                            break
                        else:
                            pass

    def planstamp(self):
        body_list = [ obj for obj in self.env.GetBodies() if 'box' in obj.GetName() ]  
        #TODO: valid only for the boxes
        logger.info('Approching the object')
        if self.herb_sim == True:
           traj = self.robot.GetActiveManipulator().PlanToEndEffectorOffset(direction=numpy.array([0,0,-1]), 
                                                                         distance=self.tsr_box_distance,
                                                                         execute=True,
                                                                         position_tolerance=0.04,
                                                                         angular_tolerance=0.15,
                                                                         timelimit=10,
                                                                         timeout=8)
           #traj = VectorFieldPlanner().PlanToEndEffectorOffset(self.robot,
                                                            #numpy.array([0,0,-1]),
                                                            #self.tsr_box_distance,
                                                            #position_tolerance=0.04,
                                                            #angular_tolerance=0.15)
           #self.robot.ExecutePath(traj, timeout=5.0) 
        else:
            while self.robot.GetActiveManipulator().GetEndEffectorTransform()[2,3] > 1.08:
                self.robot.GetActiveManipulator().MoveUntilTouch(numpy.array([0,0,-1]),  
                                                                self.tsr_box_distance, 
                                                                max_distance=self.tsr_box_distance + 2,
                                                                ignore_collisions=body_list,
                                                                max_force=7.0,
                                                                timeout=10.0,
                                                                position_tolerance=0.04,
                                                                angular_tolerance=0.15)
        if self.continuous:        
            #Mark the object as touched
            for i in range(len(self.tsrs)):
                for singletsr in self.tsrs[i]:                            
                    error = self.min_tsr[0:3,3]-singletsr[0:3,3]
                    if numpy.sqrt(numpy.dot(numpy.transpose(error), error)) < 0.0001:
                        self.nottouchedobj[i] = False
        
        logger.info('Deproaching the object')
        traj = self.robot.GetActiveManipulator().PlanToEndEffectorOffset(direction=numpy.array([0,0,1]), 
                                                                         distance=self.tsr_box_distance,
                                                                         execute=True,
                                                                         position_tolerance=0.04,
                                                                         angular_tolerance=0.15,
                                                                         timelimit=15,
                                                                         timeout=8.)
        #traj = VectorFieldPlanner().PlanToEndEffectorOffset(self.robot,
                                                            #numpy.array([0,0,1]),
                                                            #self.tsr_box_distance,
                                                            #position_tolerance=0.04,
                                                            #angular_tolerance=0.15,
                                                            #timelimit=10)
        #self.robot.ExecutePath(traj, timeout=5.0) 
        
        self.robot.GetActiveManipulator().PlanToConfiguration(self.homeconf, execute=True)
        ##with open('/tmp/traj.traj', 'w') as f:
            ##f.write(traj.serialize())
        ##print 'Saved traj to file /tmp/traj.traj'   

      
        
    def callback_override_hand(self, over_hand):
        self.over_hand = over_hand.data
 
    def callback_new_hand(self, over_hand_value):
        self.over_hand_value = over_hand_value
     
    def callback_color_obj(self, prob_goal_traj):
        count_obj = 0     
        if len(prob_goal_traj.data) > 0:
            for obj in self.env.GetBodies():
                for ref_name in REF_OBJ:                
                    if ref_name in obj.GetName(): 
                        if len(prob_goal_traj.data) > count_obj:
                            if prob_goal_traj.data[count_obj] < 0.1:
                                color = numpy.array([0.745, 0.745, 0.745]) #grey
                            elif prob_goal_traj.data[count_obj] < 0.2:
                                color = numpy.array([0.0, 1.0, 0.0]) #green
                            elif prob_goal_traj.data[count_obj] < 0.3:
                                color = numpy.array([0.2, 0.8, 0.0]) #green
                            elif prob_goal_traj.data[count_obj] < 0.4:
                                color = numpy.array([0.4, 0.7, 0.0]) #yellow
                            elif prob_goal_traj.data[count_obj] < 0.5:
                                color = numpy.array([0.5, 0.6, 0.0]) #yellow
                            elif prob_goal_traj.data[count_obj] < 0.6:
                                color = numpy.array([0.6, 0.5, 0.0]) #orange
                            elif prob_goal_traj.data[count_obj] < 0.7:
                                color = numpy.array([0.7, 0.4, 0.0]) #orange
                            elif prob_goal_traj.data[count_obj] < 0.8:
                                color = numpy.array([0.8, 0.3, 0.0]) #orange
                            elif prob_goal_traj.data[count_obj] < 0.95:
                                color = numpy.array([0.9, 0.2, 0.0]) #orange
                            else:
                                color = numpy.array([1.0, 0.0, 0.0]) #red
                            #with self.env:
                            obj.GetLinks()[0].GetGeometries()[0].SetDiffuseColor(color)
                            obj.GetLinks()[0].GetGeometries()[0].SetAmbientColor(color)
                            count_obj += 1                            
                            break
            self.robotplanforprob(prob_goal_traj.data)

 
    def callback_update_twist(self, rob_new_twist):  
        if self.envlock == True:
            return 0
        if self.min_tsr[0,3] !=  -9999:     #the selected tsr is not the fake tsr
            if rob_new_twist.data[0] != -9999:   #the goal was not reached  
                self.envlock = True
                self.restartnode = False            
                twist = numpy.array([rob_new_twist.data[0],
                                    rob_new_twist.data[1],
                                    rob_new_twist.data[2],
                                    rob_new_twist.data[3],
                                    rob_new_twist.data[4],
                                    rob_new_twist.data[5]])
        
                vlimits = self.robot.GetDOFVelocityLimits(self.robot.GetActiveDOFIndices())
                
                if self.min_tsr is None:
                    dqout, tout = util.ComputeJointVelocityFromTwist(self.robot, 
                                                                twist, 
                                                                joint_velocity_limits=vlimits,
                                                                objective=util.quadraticPlusJointLimitObjective)
                else:
                    with self.env:
                        config = self.robot.GetActiveManipulator().FindIKSolution(self.min_tsr, self.filter_options) # will return None if no config can be found
                    dqout, tout = util.ComputeJointVelocityFromTwist(self.robot, 
                                                                twist, 
                                                                joint_velocity_limits=vlimits,
                                                                objective=util.quadraticPlusJointLimitObjective)
                    #TODO: try to use the following function in order to move herb far away from the table
                    #dqout, tout = util.ComputeJointVelocityFromTwist(self.robot, 
                                                                    #twist, 
                                                                    #joint_velocity_limits=vlimits,                                                            
                                                                    #objective=util.quadraticPlusJointConf2Objective,
                                                                    #q_target=config)
                        
                ### Check collision.
                with self.env:
                    with self.robot.CreateRobotStateSaver():
                        q = self.robot.GetActiveDOFValues()
                        self.robot.SetActiveDOFValues(q + (dqout/20))  #check on herb position in 1/20 sec 
                        report = openravepy.CollisionReport()
                        if self.env.CheckCollision(self.robot, report=report):
                            raise CollisionPlanningError.FromReport(report)
                        elif self.robot.CheckSelfCollision(report=report):
                            raise SelfCollisionPlanningError.FromReport(report)
                
                self.robot.right_arm.Servo(dqout) 
                self.envlock = False
            else:
                self.envlock = True
                if self.action == 'stamp':
                    #print 'prob values when reaching the target:', self.xxx
                    #the robot touches the box
                    self.robot.right_arm.Servo(numpy.array([0., 0., 0., 0., 0., 0., 0.])) 
                    rospy.sleep(0.5) #to be sure the robot is not moving                  
                    self.planstamp()
                    rospy.sleep(1) 
                    self.tsrviz = []
                    self.tsrmsg = self.tsrchain()
                    rospy.sleep(1) #to be sure the new tsr is send 0.05>0.006
                    self.restartnode = True
                    rospy.sleep(1) #to be sure the new tsr is send 0.05>0.006
                    self.restartnode = False
                    
                if self.action == 'grasp':
                    #TODO: implement grasp action
                    pass
                self.envlock = False
       
         
    def callback_update_min_tsr(self, new_min_tsr):
        new_min_tsr = numpy.array([[new_min_tsr.data[0], new_min_tsr.data[1], new_min_tsr.data[2], new_min_tsr.data[3]],
                                   [new_min_tsr.data[4], new_min_tsr.data[5], new_min_tsr.data[6], new_min_tsr.data[7]],
                                   [new_min_tsr.data[8], new_min_tsr.data[9], new_min_tsr.data[10], new_min_tsr.data[11]],
                                   [new_min_tsr.data[12], new_min_tsr.data[13], new_min_tsr.data[14], new_min_tsr.data[15]]])
        self.cube.SetTransform(new_min_tsr)
        self.min_tsr = new_min_tsr

    def update(self, tf):  
        try:
            if self.closing==True:
                return 0
            self.publish_object_pos()
            self.publish_poses()  
            self.publish_robot_eep() 
            self.restart.publish(self.restartnode) 
            if self.continuous==True:
                self.publish_object_tsr()
            self.rate.sleep()
        except:
            print "UDATE @ ASSISTANCE.PY *******************************************************"
            print "UDATE @ ASSISTANCE.PY * Exception raised"
            print "UDATE @ ASSISTANCE.PY *******************************************************"


        
def addHumansPred(tf, humans, env, 
                  continuous=True, 
                  herb_sim=True, 
                  segway_sim=True, 
                  action='stamp'):
    humanadded = False
    
    import re
    matcher = re.compile('.*user_(\\d+).*')    
    all_tfs = tf.getFrameStrings()
    all_human_ids = []
    for frame_name in all_tfs:
        match = matcher.match(frame_name)
        if match is not None:
            all_human_ids.append(match.groups()[0])

    # adding predicion classes
    for id in all_human_ids:
        found = False
        for human in humans:
            if human.id == 'user_' + id:
                found = True
                break
        if not found:
            humanadded = True
            logger.info('Adding user %s', id)
            humans.append(Assistance('user_' + id, env, 
                                     continuous=continuous,
                                     herb_sim=herb_sim,
                                     segway_sim=segway_sim,
                                     action=action))           
    return humanadded

