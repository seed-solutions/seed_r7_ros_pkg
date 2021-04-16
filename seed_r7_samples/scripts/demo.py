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
    #rospy.loginfo("The robot was terminated")
    self.ac.cancel_goal()
#--------------------------------
class GO_TO_PLACE(State):
  def __init__(self,_place):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.place_ = _place

  def execute(self, userdata):
    rospy.loginfo('Going to Place{}'.format(self.place_))
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

    self.box1 = self.box_pose(0.6,-0.3,0.36)
    self.box2 = self.box_pose(0.7,0.3,0.76)
    self.box3 = self.box_pose(0.8,0,1.16)

    self.robot_model = rospy.get_param("/seed_r7_ros_controller/robot_model_plugin")

  def set_grasp_position(self, x, y, z, vel=1.0,direction="side"):
    self.group = moveit_commander.MoveGroupCommander("rarm_with_torso")
    self.group.set_pose_reference_frame("base_link")
    self.group.set_planner_id( "RRTConnectkConfigDefault" )
    self.group.allow_replanning( True )

    target_pose = Pose()
    if(direction == "side"):
      self.group.set_end_effector_link("r_eef_grasp_link")
      quat = tf.transformations.quaternion_from_euler(0,0,0)
    elif(direction == "top"): 
      self.group.set_end_effector_link("r_eef_pick_link")
      quat = tf.transformations.quaternion_from_euler(-1.57,0.79,0)

    target_pose.orientation.x = quat[0]
    target_pose.orientation.y = quat[1]
    target_pose.orientation.z = quat[2]
    target_pose.orientation.w = quat[3]

    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z

    self.group.set_pose_target(target_pose)
    self.group.set_max_velocity_scaling_factor(vel)
    plan = self.group.plan()
    if type(plan) is tuple: # for noetic
        plan = plan[1]

    if(len(plan.joint_trajectory.points)==0):
      rospy.logwarn("IK can't be solved")
      self.group.clear_pose_targets()
      return 'aborted'
    else: 
      self.group.execute(plan)
      return 'succeeded'

  def set_lifter_position(self, x, z, vel=1.0):
    self.group = moveit_commander.MoveGroupCommander("torso")
    self.group.set_pose_reference_frame("base_link")
    self.group.set_end_effector_link("body_link")
    if("typef" in self.robot_model):
      distance_body_lifter = 1.065 - 0.92
    elif("typeg" in self.robot_model):
      distance_body_lifter = 0.994 - 0.857

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

  def wait_for_state_update(self, box_name,box_is_known=False, box_is_attached=False, timeout=4):
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      is_known = box_name in scene.get_known_object_names()

      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return 'succeeded'

      rospy.sleep(0.1)
      seconds = rospy.get_time()

    return 'aborted'

  def box_pose(self, x, y, z):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z
    return box_pose

  def add_objects(self):
    self.scene.add_box("shelf1", self.box_pose(0.8,0,0.3), size=(0.5, 1.0, 0.01))
    self.scene.add_box("shelf2", self.box_pose(0.8,0,0.7), size=(0.5, 1.0, 0.01))
    self.scene.add_box("shelf3", self.box_pose(0.8,0,1.1), size=(0.5, 1.0, 0.01))

    self.scene.add_box("wall1", self.box_pose(0.8,0.5,0.75), size=(0.5, 0.01, 1.5))
    self.scene.add_box("wall2", self.box_pose(0.8,-0.5,0.75), size=(0.5, 0.01, 1.5))

    self.scene.add_box("box1", self.box1, size=(0.05, 0.1, 0.1))
    self.scene.add_box("box2", self.box2, size=(0.05, 0.05, 0.1))
    self.scene.add_box("box3", self.box3, size=(0.05, 0.05, 0.1))

    return 'succeeded'

  def remove_objects(self):
    self.scene.remove_world_object()
    return 'succeeded'

  def attach_objects(self,object_name):
    self.group.set_end_effector_link("r_eef_pick_link")
    eef_link = self.group.get_end_effector_link()
    grasping_group = 'rhand'
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box(eef_link, object_name, touch_links=touch_links)

    return self.wait_for_state_update(object_name,box_is_attached=True, box_is_known=False, timeout=4)

  def detach_objects(self, object_name):
    self.group.set_end_effector_link("r_eef_pick_link")
    eef_link = self.group.get_end_effector_link()

    self.scene.remove_attached_object(eef_link, name=object_name)

    return self.wait_for_state_update(object_name,box_is_known=True, box_is_attached=False, timeout=4)

#---------------------------------
class MANIPULATE(State):
  def __init__(self,x,y,z,vel=1.0,direction="side"):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.x = x
    self.y = y
    self.z = z
    self.vel = vel
    self.direction = direction

  def execute(self, userdata):
    rospy.loginfo('Manipulate at ({},{},{}) in scale velocity {}'.format(self.x,self.y,self.z,self.vel))
    if(mc.set_grasp_position(self.x,self.y,self.z,self.vel,self.direction) 
      == 'succeeded'):return 'succeeded'
    else: return 'aborted'

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

class UPDATE_OBJECTS(State):
  def __init__(self,action):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.action = action

  def execute(self, userdata):
    if(self.action == 'add'): 
      print('add objects')
      if(mc.add_objects() == 'succeeded'):return 'succeeded'
      else: return 'aborted'
    elif(self.action == 'remove'): 
      print('remove objects')
      if(mc.remove_objects() == 'succeeded'):return 'succeeded'
      else: return 'aborted'
      
class PICK(State):
  def __init__(self,object_name):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.object_name = object_name

  def execute(self, userdata):
    if(mc.attach_objects(self.object_name) == 'succeeded'):
      if(hc.grasp()): return 'succeeded'
      else: return 'aborted'
    else: return 'aborted'

class PLACE(State):
  def __init__(self,object_name):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.object_name = object_name

  def execute(self, userdata):
    if(mc.detach_objects(self.object_name) == 'succeeded'):
      if(hc.release()): return 'succeeded'
      else: return 'aborted'
    else: return 'aborted'

#==================================
#==================================
if __name__ == '__main__':
  rospy.init_node('scenario_node')

  na = NaviAction()
  hc = HandController()
  mc = MoveitCommand()

  go_to_shelf = StateMachine(outcomes=['succeeded','aborted'])
  with go_to_shelf:
    StateMachine.add('DOWN LIFTER', MOVE_LIFTER(0,0.6),\
      transitions={'succeeded':'MOVE','aborted':'aborted'})
    StateMachine.add('MOVE', GO_TO_PLACE(1),\
      transitions={'succeeded':'succeeded','aborted':'aborted'})

  go_to_start_point = StateMachine(outcomes=['succeeded','aborted'])
  with go_to_start_point:
    StateMachine.add('DOWN LIFTER', MOVE_LIFTER(0,0.6),\
      transitions={'succeeded':'MOVE','aborted':'aborted'})
    StateMachine.add('MOVE', GO_TO_PLACE(0),\
      transitions={'succeeded':'succeeded','aborted':'aborted'})

  pick_place_1 = StateMachine(outcomes=['succeeded','aborted'])
  x = mc.box1.pose.position.x
  y = mc.box1.pose.position.y
  z = mc.box1.pose.position.z
  with pick_place_1:
    StateMachine.add('PICK MOTION', MANIPULATE(x, y, z, direction="top"), \
      transitions={'succeeded':'PICK','aborted':'aborted'})
    StateMachine.add('PICK', PICK('box1'),\
      transitions={'succeeded':'PLACE MOTION','aborted':'aborted'})
    StateMachine.add('PLACE MOTION', MANIPULATE(0.6, -0.4, z + 0.4, direction="top"), \
      transitions={'succeeded':'PLACE','aborted':'aborted'})   
    StateMachine.add('PLACE', PLACE('box1'),\
      transitions={'succeeded':'succeeded','aborted':'aborted'})

  pick_place_2 = StateMachine(outcomes=['succeeded','aborted'])
  x = mc.box2.pose.position.x
  y = mc.box2.pose.position.y
  z = mc.box2.pose.position.z
  with pick_place_2:
    StateMachine.add('PICK MOTION', MANIPULATE(x, y, z), \
      transitions={'succeeded':'PICK','aborted':'aborted'})
    StateMachine.add('PICK', PICK('box2'),\
      transitions={'succeeded':'PLACE MOTION','aborted':'aborted'})
    StateMachine.add('PLACE MOTION', MANIPULATE(0.6, -0.2, z), \
      transitions={'succeeded':'PLACE','aborted':'aborted'})    
    StateMachine.add('PLACE', PLACE('box2'),\
      transitions={'succeeded':'succeeded','aborted':'aborted'})

  pick_place_3 = StateMachine(outcomes=['succeeded','aborted'])
  x = mc.box3.pose.position.x
  y = mc.box3.pose.position.y
  z = mc.box3.pose.position.z
  with pick_place_3:
    StateMachine.add('PICK MOTION', MANIPULATE(x, y, z), \
      transitions={'succeeded':'PICK','aborted':'aborted'})
    StateMachine.add('PICK', PICK('box3'),\
      transitions={'succeeded':'PLACE MOTION','aborted':'aborted'})
    StateMachine.add('PLACE MOTION', MANIPULATE(0.6, 0.0, z - 0.4), \
      transitions={'succeeded':'PLACE','aborted':'aborted'})  
    StateMachine.add('PLACE', PLACE('box3'),\
      transitions={'succeeded':'succeeded','aborted':'aborted'})

  scenario_play = StateMachine(outcomes=['succeeded','aborted'])
  with scenario_play:
    StateMachine.add('GO TO SHELF', go_to_shelf,\
      transitions={'succeeded':'ADD OBJECTS','aborted':'aborted'})
    StateMachine.add('ADD OBJECTS', UPDATE_OBJECTS('add'),\
      transitions={'succeeded':'PICK and PLACE 1','aborted':'aborted'})
    StateMachine.add('PICK and PLACE 1', pick_place_1,\
      transitions={'succeeded':'PICK and PLACE 2','aborted':'aborted'})
    StateMachine.add('PICK and PLACE 2', pick_place_2,\
      transitions={'succeeded':'PICK and PLACE 3','aborted':'aborted'})
    StateMachine.add('PICK and PLACE 3', pick_place_3,\
      transitions={'succeeded':'INITIALIZE','aborted':'aborted'})
    StateMachine.add('INITIALIZE', INIT_POSE(),\
      transitions={'succeeded':'REMOVE OBJECTS','aborted':'aborted'})
    StateMachine.add('REMOVE OBJECTS', UPDATE_OBJECTS('remove'),\
      transitions={'succeeded':'GO TO START POINT','aborted':'aborted'})
    StateMachine.add('GO TO START POINT', go_to_start_point,\
      transitions={'succeeded':'GO TO SHELF','aborted':'aborted'})

  sis = smach_ros.IntrospectionServer('server_name',scenario_play,'/SEED-Noid-Mover Scenario Play')
  sis.start()
  scenario_play.execute()
  sis.stop()

