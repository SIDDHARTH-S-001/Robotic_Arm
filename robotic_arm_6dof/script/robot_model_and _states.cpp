#include<ros/ros.h>
#include<moveit/robot_model_loader/robot_model_loader.h>
#include<moveit/robot_state/robot_state.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_model_and_states"); // Start a ros node
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Most higher-level componenets will rreturn a shared pointer to RobotModel.
    // RobotModelLoader
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model Frame: %s", kinematic_model->getModelFrame().c_str());
    
    // We can use moveit core to construct a RobotState that will maintain the configuration of the robot.
    // The JointModelGroup represents model for a particular group.
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("robot");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    // Get Joint Values
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i=; i<joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str, joint_values[i]);
    }

    // Joint Limits

    // setJointGroupPosition doesn't enforce joint limits by itself but a call to enforceBounds() will do it.
    // set one joint beyond limits
    joint[1] = 2.57;
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

    // Check if any joint is outside limits
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    // Enforce the joint limits for this state and check again
    kinematic_state->enforceBounds();
    ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    // Forward Kinematics
    // This can be done by finding the pose of the most distant jonit from base in the robot.
    // In case of our Robot its MG996R_v5__5__1
    kinematic_state->setToRandomPositions(joint_model_group);
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("MG996R_v5__5__1");

    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

    // Inverse Kinematics
    // To solve inverse-kinematics we need the following
    // 1) desired pose of the ned_effector (by default this is the last link in the chain).
    // we have already computed end_effector_state in the previous state.
    // 2) Timeout 0.1s.

    double timeout = 0.1;
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

    if(found_ik)
    {
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i=0; i<joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }
    
    else
    {
        ROS_INFO("Didn't find IK Solution");
    }

    // Jacobian
    // Jcaobian can be gotten from moveit core RobotState.
    Eigen::Vector3d reference_point_positions(0.0, 0.0, 0.0);
    Eigen::Matrix3d jacobian;
    kinematic_state->getJacobian(joint_model_group, 
                                kinematic_state->getLinkModel(joint_model_group->getLinkModelName().back()),
                                reference_point_position, jacobian);
    ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

    ros::shutdown();
    return 0;
}
