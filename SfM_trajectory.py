#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np
from pyquaternion import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker

pub = rospy.Publisher('poses', PoseArray, queue_size=50)
pub_marker = rospy.Publisher('visualization_marker', Marker, queue_size=50)

def create_marker(x, y, z):
  marker = Marker()
  marker.header.frame_id = "world"
  marker.header.stamp = rospy.Time.now()
  marker.ns = "my_namespace"
  marker.id = 0
  marker.type = Marker.SPHERE
  marker.action = Marker.ADD
  marker.pose.position.x = x
  marker.pose.position.y = y
  marker.pose.position.z = z
  marker.pose.orientation.x = 0.0
  marker.pose.orientation.y = 0.0
  marker.pose.orientation.z = 0.0
  marker.pose.orientation.w = 1.0
  marker.scale.x = 0.1
  marker.scale.y = 0.1
  marker.scale.z = 0.1
  marker.color.a = 1.0
  marker.color.r = 1.0
  marker.color.g = 1.0
  marker.color.b = 0.0
  return marker


def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def look_at(camera_position, camera_target, up_vector):
    vector = camera_target - camera_position
    # vector = camera_position - camera_target
    vector = vector / np.linalg.norm(vector)
    vector2 = np.cross(vector, up_vector)
    vector3 = np.cross(vector, vector2)
    vector2 = vector2 / np.linalg.norm(vector2)
    vector3 = vector3 / np.linalg.norm(vector3)
    return np.array([
		[vector2[0], vector2[1], vector2[2], -np.dot(vector2, camera_position)],
		[vector3[0], vector3[1], vector3[2], -np.dot(vector3, camera_position)],
		[vector[0], vector[1], vector[2], np.dot(vector, camera_position)],
        [0.0, 0.0, 0.0, 1.0]
	])

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def my_go_to_pose_goal(self):
    # Set object position and visualize using a marker
    object_position = [0.4, 0.85, -0.5]
    obj = create_marker(object_position[0], object_position[1], object_position[2])
    pub_marker.publish(obj)
    rospy.sleep(0.1)
    # Create circle of points around object
    # parameters for circle path
    radius = 0.3
    offset_angle = 115 #degrees
    centre = object_position #x, y, z
    no_of_points = 50

    theta = np.linspace(0, 2 * np.pi, no_of_points+1)
    phi = np.deg2rad(offset_angle + 90)
    x = centre[0] + radius * np.cos(theta) 
    y = centre[1] + radius * np.sin(theta) * np.sin(phi)
    z = centre[2] + radius * np.sin(theta) * np.cos(phi)
    # remove last point (duplicate of the 1st point)
    x=x[:len(x)-1]; y=y[:len(y)-1]; z=z[:len(z)-1]
    # List of all points
    xyz = np.stack((x, y, z), axis=1)
    # Get orientation quaternion for each point so it looks at the object
    quats = np.empty((0,4))
    for row in range(len(xyz)):
      cam_point = xyz[row]
      mat = look_at(cam_point, object_position, np.array([0,0,1]))
      mat = mat[:3,:3]
      q = (Quaternion(matrix=mat))
      q = np.array([q[1], q[2], q[3], q[0]])
      q = quaternion_multiply(q, np.array([0,-0.7071068,0,0.7071068]))
      q = quaternion_multiply(q, np.array([-0.7071068,0,0,0.7071068]))
      q[3] = -q[3]
      quats = np.vstack((quats, q))
    # Delete half of the circle to get an arc
    keep = np.ones(xyz.shape[0], dtype=bool)
    for pos, _ in enumerate(xyz):
        if object_position[1] <= 0:
          if offset_angle <= 90:
            if object_position[1]-xyz[pos][1] >= -(0.5*np.sin(offset_angle*np.pi/180.))*radius:
                keep[pos] = False
          else:
            if object_position[2]-xyz[pos][2] <= (0.3*np.sin(offset_angle*np.pi/180.))*radius:
                keep[pos] = False
        elif object_position[1] > 0:
          if offset_angle <= 90:
            if xyz[pos][1]-object_position[1] >= -(0.5*np.sin(offset_angle*np.pi/180.))*radius:
                keep[pos] = False
          else:
            if xyz[pos][1]-object_position[1] <= (0.3*np.sin(offset_angle*np.pi/180.))*radius:
                keep[pos] = False
    xyz = xyz[keep]
    quats = quats[keep]
    # Visualize proposed poses as an rviz PoseArray
    my_posearray = PoseArray()
    my_posearray.header.frame_id = 'world'
    my_posearray.header.stamp = rospy.Time.now()
    for row in range(len(xyz)):
      pose_goal = Pose()
      pose_goal.orientation.x = quats[row][0]
      pose_goal.orientation.y = quats[row][1]
      pose_goal.orientation.z = quats[row][2]
      pose_goal.orientation.w = quats[row][3]
      pose_goal.position.x = xyz[row][0]
      pose_goal.position.y = xyz[row][1]
      pose_goal.position.z = xyz[row][2]
      my_posearray.poses.append(pose_goal)
    pub.publish(my_posearray)
    rospy.sleep(0.1)


    group = self.group
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    for row in range(len(xyz)):
      pose_goal = Pose()
      pose_goal.orientation.x = quats[row][0]
      pose_goal.orientation.y = quats[row][1]
      pose_goal.orientation.z = quats[row][2]
      pose_goal.orientation.w = quats[row][3]
      pose_goal.position.x = xyz[row][0]
      pose_goal.position.y = xyz[row][1]
      pose_goal.position.z = xyz[row][2]
      group.set_pose_target(pose_goal)
      ## Now, we call the planner to compute the plan and execute it.
      plan = group.go(wait=True)
      # Calling `stop()` ensures that there is no residual movement
      group.stop()
      # It is always good to clear your targets after planning with poses.
      # Note: there is no equivalent function for clear_joint_value_targets()
      group.clear_pose_targets()
      ## END_SUB_TUTORIAL

      # For testing:
      # Note that since this section of code will not be included in the tutorials
      # we use the class variable rather than the copied state variable
      current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


def main():
  try:
    print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander (press ctrl-d to exit) ..."
    raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    print "============ Press `Enter` to execute a movement using a pose goal ..."
    raw_input()
    tutorial.my_go_to_pose_goal()

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/kinetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/kinetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/kinetic/api/rospy/html/