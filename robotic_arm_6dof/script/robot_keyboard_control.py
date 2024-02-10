#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
import sys

def move_to_target_pose():
    # Initialize MoveIt!
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_to_target_pose_node', anonymous=True)

    # Create a move group instance
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander('robot')

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 20)

    # Get the current pose of the end-effector link
    end_effector_link = group.get_end_effector_link()
    print("End-effector link: ", end_effector_link)
    target_pose = Pose()
    # group.set_planning_frame(end_effector_link)

    # while not rospy.is_shutdown():
    #     # Define the target pose
    #     target_pose = PoseStamped()
    #     # target_pose = Pose()
    #     target_pose.header.frame_id = robot.get_planning_frame()
    #     target_pose.header.frame_id = end_effector_link
    #     print('q: x+0.1   |   w:x-0.1')
    #     print('a: y+0.1   |   s:y-0.1')
    #     print('z: z+0.1   |   x:z-0.1')
    #     key = input("Enter a command: ")
    #     if key == 'q':
    #         target_pose.pose.position.x +=0.1
    #     elif key == 'w':
    #         target_pose.pose.position.x +=0.1
    #     elif key == 'a':
    #         target_pose.pose.position.y +=0.1
    #     elif key == 's':
    #         target_pose.pose.position.y -=0.1
    #     elif key == 'z':
    #         target_pose.pose.position.z +=0.1
    #     elif key == 'x':
    #         target_pose.pose.position.z -=0.1
            
    # target_pose.header.frame_id = robot.get_planning_frame()
    # target_pose.header.frame_id = end_effector_link
    target_pose.position.x = 0.5
    target_pose.position.y = 0.25
    target_pose.position.z = 0.75
    target_pose.orientation.w = 0.75
    # print(target_pose)

    # Set the target pose for the end-effector link
    # group.set_named_target('up')

    # Plan and execute the motion
    group.plan()
    group.go(wait=True)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size = 20)
    # group.set_random_target()
    # group.stop()
    # group.clear_pose_targets()        

    # current_pose = group.get_current_pose().pose

    # Clean up
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        move_to_target_pose()
    except rospy.ROSInterruptException:
        pass
