#!/usr/bin/env python3
# from __future__ import print_function
from six.moves import input # Warning on this line can be ignored.

import rospy
import copy
import sys

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

try:
    from math import pi, tau, dist, fabs, cosh
except: # for python 2 compatibility
    from math import pi, sqrt, fabs, cos

    tau = 2*pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i)**2 for p_i, q_i in zip(p, q)))


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """

    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index] > tolerance):
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(actual)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, y0))
        # phi = angle between orientations
        cos_phi_half = fabs(qw1*qw0 + qx1*qx0 + qy1*qy0 + qz1*qz0)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)


class MoveGroupPythonInterface(object):
    def __init__(self):
        super(MoveGroupPythonInterface, self).__init__()

        # First instantiate Moveit Commander and a Rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_py_interface', anonymous=True)

        # Create a Robot Commander Object -> Provides information about Robot's kinematic model and Robot states
        robot = moveit_commander.RobotCommander()

        # Instantiate PlanningSceneInterface
        scene = moveit_commander.PlanningSceneInterface()

        # Instiate move-group-commmander object. This is an Interface to the planning group.
        group_name = 'robot'
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # Create a Display Trajectory for RVIZ.
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 20)

        # Getting Info
        # Reference Frame
        planning_frame  = move_group.get_planning_frame()
        print("======== Planning Frame: %s" %planning_frame) # % in print statement is used for string substitution

        # Name of eef link
        eef_link = move_group.get_end_effector_link()
        print("======== End-Effector-Link: %s" %eef_link)

        # Getting Group Names
        group_names = robot.get_group_names()
        print("======== Available Planning Groups: %s" %group_names)

        # State of the robot
        print("======== Robot State =======")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        move_group = self.move_group

        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def go_to_pose_goal(self):
        move_group = self.move_group

        # Planning to a pose-goal
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.25
        pose_goal.position.y = 0.175
        pose_goal.position.z = 0.2

        move_group.set_pose_target(pose_goal)

        # Now we call the planner to plan and execute
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


    def plan_cartesian_path(self, scale=1):
        # Planning a Cartesian Path by specifying a list of way-points
        move_group = self.move_group
        
        waypoints = []
        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1 # First Move Up (Z)
        wpose.position.y += scale * 0.2 # and Sideways (Y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        #                                        waypoints to follow, eef_step, jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction 


    def display_trajectory(self, plan):
        # Displaying a Trajectory
        # A DisplayTrajectory msg has two primary fields, trajectory_start and trajectory. 
        # We populate the trajectory_start with our current robot state to copy over any AttachedCollisionObjects 
        # and add our plan to the trajectory.

        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def wait_for_state_update(self, box_is_known = False, box_is_attached = False, timeout = 4):
        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Check if the box is an attached object
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Note that attaching thr box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Check if they are in any expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
            
            # Give sleep to give other threads some time on processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        
        # If we exited while loop without returning then we timed out.
        return False

    def add_box(self, timeout=4):
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "suction_cup_v2_1"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11  # above the suction cup frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name    
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the robot's vaccum cup. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the robot
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "eef"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )

def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin by setting up the moveit_commander ..."
        )
        interface = MoveGroupPythonInterface()

        input(
            "============ Press `Enter` to execute a movement using a joint state goal ..."
        )
        interface.go_to_joint_state()

        input("============ Press `Enter` to execute a movement using a pose goal ...")
        interface.go_to_pose_goal()

        input("============ Press `Enter` to plan and display a Cartesian path ...")
        cartesian_plan, fraction = interface.plan_cartesian_path()

        input(
            "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        )
        interface.display_trajectory(cartesian_plan)

        input("============ Press `Enter` to execute a saved path ...")
        interface.execute_plan(cartesian_plan)

        input("============ Press `Enter` to add a box to the planning scene ...")
        interface.add_box()

        input("============ Press `Enter` to attach a Box to the robot ...")
        interface.attach_box()

        input(
            "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        )
        cartesian_plan, fraction = interface.plan_cartesian_path(scale=-1)
        interface.execute_plan(cartesian_plan)

        input("============ Press `Enter` to detach the box from the robot robot ...")
        interface.detach_box()

        input(
            "============ Press `Enter` to remove the box from the planning scene ..."
        )
        interface.remove_box()

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##