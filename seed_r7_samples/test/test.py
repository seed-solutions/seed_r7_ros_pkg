#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import rospy
##-- for smach
from smach import State,StateMachine
import smach_ros
##-- for navigation
import yaml
import tf
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
##-- for find pkg
import rospkg
##-- for moveit
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
import geometry_msgs.msg
##-- for hand control
from seed_r7_ros_controller.srv import*


###########################################
class NaviAction:
  def __init__(self):
    rospack = rospkg.RosPack()
    rospack.list() 
    path = rospack.get_path('seed_r7_samples')
    with open(path + '/config/waypoints.yaml') as f:
        self.config = yaml.load(f)
    rospy.on_shutdown(self.shutdown)
    self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    while not self.ac.wait_for_server(rospy.Duration(5)):
      rospy.loginfo("Waiting for the move_base action server to come up")
    rospy.loginfo("The server comes up");
    self.goal = MoveBaseGoal()

  def set_goal(self,_number):
    rospy.on_shutdown(self.shutdown)

    rev = dict(self.config[_number]) #List to Dictionary

    self.goal.target_pose.header.frame_id = 'map'
    self.goal.target_pose.header.stamp = rospy.Time.now()
    self.goal.target_pose.pose.position.x = rev['pose']['position']['x']
    self.goal.target_pose.pose.position.y = rev['pose']['position']['y']
    self.goal.target_pose.pose.position.z = rev['pose']['position']['z']
    self.goal.target_pose.pose.orientation.x = rev['pose']['orientation']['x']
    self.goal.target_pose.pose.orientation.y = rev['pose']['orientation']['y']
    self.goal.target_pose.pose.orientation.z = rev['pose']['orientation']['z']
    self.goal.target_pose.pose.orientation.w = rev['pose']['orientation']['w']

    rospy.loginfo('Sending goal')
    self.ac.send_goal(self.goal)
    succeeded = self.ac.wait_for_result(rospy.Duration(60));
    state = self.ac.get_state();
    if succeeded:
      rospy.loginfo("Succeed")
      return 'succeeded'
    else:
      rospy.loginfo("Failed")
      return 'aborted'

  def shutdown(self):
    rospy.loginfo("The robot was terminated")
    self.ac.cancel_goal()
#--------------------------------
class GO_TO_PLACE(State):
  def __init__(self,_place):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.place_ = _place

  def execute(self, userdata):
    rospy.loginfo('Going to Place'.format(self.place_))
    if(na.set_goal(self.place_) == 'succeeded'):return 'succeeded'
    else: return 'aborted' 

###########################################
class HandController:
  def __init__(self):
    rospy.loginfo('waiting service')
    rospy.wait_for_service('/seed_r7_ros_controller/hand_control')

  def grasp(self):
    try:
        service = rospy.ServiceProxy('/seed_r7_ros_controller/hand_control', HandControl)
        response = service(0,'grasp',100)
        return True
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: {}'.format(e))
        return False

  def release(self):
    try:
        service = rospy.ServiceProxy('/seed_r7_ros_controller/hand_control', HandControl)
        response = service(0,'release',100)
        return True
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: {}'.format(e))
        return False



###########################################
class MoveitCommand:
  def __init__(self):
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()

    self.group = moveit_commander.MoveGroupCommander("rarm_with_torso")
    self.group.set_pose_reference_frame("base_link")

  def set_lifter_position(self, x, z, vel=1.0):
    self.group = moveit_commander.MoveGroupCommander("torso")
    self.group.set_pose_reference_frame("base_link")
    self.group.set_end_effector_link("body_link")
    distance_body_lifter = 1.065 - 0.92

    listener = tf.TransformListener()

    while True:
      try:
        #listener.waitForTransform('base_link', 'body_link',rospy.Time.now(), rospy.Duration(6.0))
        (position, quaternion) = listener.lookupTransform('base_link', 'body_link', rospy.Time(0) )
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
      
      if(len(position)>0): break

    target_pose = Pose()

    target_pose.orientation.x = 0
    target_pose.orientation.y = 0
    target_pose.orientation.z = 0
    target_pose.orientation.w = 1

    target_pose.position.x = x
    target_pose.position.y = 0
    target_pose.position.z = z + distance_body_lifter

    self.group.set_start_state_to_current_state()
    self.group.set_pose_target(target_pose)
    self.group.set_max_velocity_scaling_factor(vel)
    plan = self.group.plan()
    if type(plan) is tuple: # for noetic
        plan = plan[1]

    if(len(plan.joint_trajectory.points)==0):
      rospy.logwarn("can't be solved lifter ik")
      self.group.clear_pose_targets()
      return 'aborted'
    else: 
      self.group.execute(plan)
      return 'succeeded'

  def set_initial_pose(self):
    self.group = moveit_commander.MoveGroupCommander("upper_body")
    self.group.set_named_target("reset-pose")
    plan = self.group.plan()
    if type(plan) is tuple: # for noetic
        plan = plan[1]
    self.group.go()

    if(len(plan.joint_trajectory.points)==0):
      rospy.logwarn("IK can't be solved")
      self.group.clear_pose_targets()
      return 'aborted'
    else: 
      self.group.execute(plan)
      return 'succeeded'

#---------------------------------
class MOVE_LIFTER(State):
  def __init__(self,x,z,vel=1.0):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.x = x
    self.z = z
    self.vel = vel

  def execute(self, userdata):
    rospy.loginfo('Move Lifter at ({},{}) in scale velocity {}'.format(self.x,self.z,self.vel))
    if(mc.set_lifter_position(self.x,self.z,self.vel) == 'succeeded'):return 'succeeded'
    else: return 'aborted'

#---------------------------------
class INIT_POSE(State):
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','aborted'])

  def execute(self, userdata):
    rospy.loginfo('initialize wholebody')

    if(mc.set_initial_pose() == 'succeeded'):return 'succeeded'
    else: return 'aborted'


class FINISH(State):
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','aborted'])

  ## @brief 遷移実行
  # @param userdata 前のステートから引き継がれた変数。今回は使用しない
  # @return succeededのみ
  def execute(self, userdata):
    rospy.loginfo('FINISHED')
    return 'succeeded'

#==================================
#==================================
if __name__ == '__main__':
  rospy.init_node('test_node')

  na = NaviAction()
  hc = HandController()
  mc = MoveitCommand()

  scenario_play = StateMachine(outcomes=['succeeded','aborted'])
  with scenario_play:
    StateMachine.add('DOWN LIFTER', MOVE_LIFTER(0,0.6),\
      transitions={'succeeded':'FINISH','aborted':'aborted'})
    StateMachine.add('FINISH', FINISH(),\
      transitions={'succeeded':'succeeded','aborted':'aborted'})
  
  sis = smach_ros.IntrospectionServer('server_name',scenario_play,'/SEED-Noid-Mover Scenario Play')
  sis.start()
  scenario_play.execute()
  time.sleep(0.1)
  rospy.loginfo('end')
  sis.stop()
