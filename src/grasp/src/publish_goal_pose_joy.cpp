#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

# define PI 3.14159265359

class JoyControlRobot : public rclcpp::Node {
public:

    //geometry_msgs::msg::Pose target_pose;
    geometry_msgs::msg::Pose cube1_pose;
    geometry_msgs::msg::Pose cube2_pose;
    geometry_msgs::msg::Pose target_pose;
    geometry_msgs::msg::Quaternion q_current;
    geometry_msgs::msg::Quaternion q_cube;
    double roll_current, pitch_current, yaw_current;
    double roll_cube, pitch_cube, yaw_cube;

    JoyControlRobot()
        : Node("joy_control_robot") {
        // Create joystick subscriber
        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/isaac_tf", 10,
            std::bind(&JoyControlRobot::tfCallback, this, std::placeholders::_1));

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyControlRobot::joyCallback, this, std::placeholders::_1));

        vision_grasp_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/grasp_target_pose", 10,
            std::bind(&JoyControlRobot::visionGraspCallback, this, std::placeholders::_1));
    }

    void initializeMoveGroup() {
        // Ensure shared_from_this() is called only after the node is managed by std::shared_ptr
        arm_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm");

        gripper_move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "gripper");

        // Set the initial target pose to the current pose
        current_pose = arm_move_group->getCurrentPose();
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized with current pose.");
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (!arm_move_group) {
            RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not initialized yet.");
            return;
        }

        if (msg->buttons[0] == 1){ //X button, move the EE to the Cube1 position
        
            current_pose = arm_move_group->getCurrentPose(); //pose of ee_gripper_link
      
            RCLCPP_INFO(this->get_logger(), "Current EE Pose:");
            RCLCPP_INFO(this->get_logger(), "Position - x: %f, y: %f, z: %f",
                    current_pose.pose.position.x,
                    current_pose.pose.position.y,
                    current_pose.pose.position.z);
            RCLCPP_INFO(this->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f",
                    current_pose.pose.orientation.x,
                    current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z,
                    current_pose.pose.orientation.w);


            q_current = current_pose.pose.orientation;
            tf2::Quaternion tf_quat_current(q_current.x, q_current.y, q_current.z, q_current.w);
            tf2::Matrix3x3(tf_quat_current).getRPY(roll_current, pitch_current, yaw_current);
            RCLCPP_INFO(this->get_logger(), "Current EE Roll: %f, Pitch: %f, Yaw: %f", roll_current, pitch_current, yaw_current);

            RCLCPP_INFO(this->get_logger(), "Cube1 Pose:");
            RCLCPP_INFO(this->get_logger(), "Position - x: %f, y: %f, z: %f",
                    cube1_pose.position.x,
                    cube1_pose.position.y,
                    cube1_pose.position.z);
            RCLCPP_INFO(this->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f",
                    cube1_pose.orientation.x,
                    cube1_pose.orientation.y,
                    cube1_pose.orientation.z,
                    cube1_pose.orientation.w);

            q_cube = cube1_pose.orientation;
            tf2::Quaternion tf_quat_cube(q_cube.x, q_cube.y, q_cube.z, q_cube.w);
            tf2::Matrix3x3(tf_quat_cube).getRPY(roll_cube, pitch_cube, yaw_cube);
            RCLCPP_INFO(this->get_logger(), "Current EE Roll: %f, Pitch: %f, Yaw: %f", roll_cube, pitch_cube, yaw_cube);
        
            tf2::Quaternion tf_quat;
            tf_quat.setRPY(roll_current, pitch_current, yaw_current + yaw_cube);

            // move above the cube
            target_pose.position.x = cube1_pose.position.x;
            target_pose.position.y = cube1_pose.position.y;
            target_pose.position.z = current_pose.pose.position.z;
            target_pose.orientation.x = tf_quat.x();
            target_pose.orientation.y = tf_quat.y();
            target_pose.orientation.z = tf_quat.z();
            target_pose.orientation.w = tf_quat.w();

            pose_target_plan_and_execute(target_pose);

            // lower the arm
            target_pose.position.z = cube1_pose.position.z;

            pose_target_plan_and_execute(target_pose);

        }
        else if (msg->buttons[1] == 1){ //O button, move the EE to the Cube2 position
            current_pose = arm_move_group->getCurrentPose(); //pose of ee_gripper_link

            q_current = current_pose.pose.orientation;
            tf2::Quaternion tf_quat_current(q_current.x, q_current.y, q_current.z, q_current.w);
            tf2::Matrix3x3(tf_quat_current).getRPY(roll_current, pitch_current, yaw_current);

            q_cube = cube2_pose.orientation;
            tf2::Quaternion tf_quat_cube(q_cube.x, q_cube.y, q_cube.z, q_cube.w);
            tf2::Matrix3x3(tf_quat_cube).getRPY(roll_cube, pitch_cube, yaw_cube);
        
            tf2::Quaternion tf_quat;
            tf_quat.setRPY(roll_current, pitch_current, yaw_current + yaw_cube);

            // move above the cube
            target_pose.position.x = cube2_pose.position.x;
            target_pose.position.y = cube2_pose.position.y;
            target_pose.position.z = current_pose.pose.position.z;
            target_pose.orientation.x = tf_quat.x();
            target_pose.orientation.y = tf_quat.y();
            target_pose.orientation.z = tf_quat.z();
            target_pose.orientation.w = tf_quat.w();

            pose_target_plan_and_execute(target_pose);

            // lower the arm
            target_pose.position.z = cube2_pose.position.z;

            pose_target_plan_and_execute(target_pose);
        }
        else if (msg->buttons[2] == 1){ //triangle button, move to the "ready" position
            named_target_plan_and_execute("ready");
        }
        else if (msg->buttons[3] == 1){ //square button, move to the "rest" position
            named_target_plan_and_execute("rest");
        }
        else if (msg->buttons[4] == 1){ //L1 button, close the gripper
            joint_value_target_plan_and_execute(45.0f);
        }
        else if (msg->buttons[5] == 1){ //R1 button, open the gripper
            joint_value_target_plan_and_execute(0.0f);
        }
    }

    void pose_target_plan_and_execute(const geometry_msgs::msg::Pose target_pose){
        // Plan to the goal pose
        arm_move_group->setPoseTarget(target_pose);
        RCLCPP_INFO(this->get_logger(), "Planning to the goal pose...");
        auto success = (arm_move_group->plan(my_plan_) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Plan successful. Executing the motion...");
            auto execution_success = (arm_move_group->execute(my_plan_) == moveit::core::MoveItErrorCode::SUCCESS);

            if (execution_success)
            {
                RCLCPP_INFO(this->get_logger(), "Motion execution successful!");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Motion execution failed.");
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Planning failed. Unable to find a valid plan.");
        }        
    } 

    void named_target_plan_and_execute(const std::string GROUP_STATE_NAME){
        bool success = arm_move_group->setNamedTarget(GROUP_STATE_NAME);    
        if (success) {
            // Plan and execute the motion
            auto execution_success = arm_move_group->move();
            if (execution_success) {
                RCLCPP_INFO(this->get_logger(), "Motion execution to group state '%s' was successful.", GROUP_STATE_NAME.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Motion execution to group state '%s' failed.", GROUP_STATE_NAME.c_str());
            }
        }
        else{
            RCLCPP_ERROR(this->get_logger(), "Failed to set the named target: %s", GROUP_STATE_NAME.c_str());
        }
    }

    void joint_value_target_plan_and_execute(float angle){
        // Get current joint values
        std::vector<double> joint_values = gripper_move_group->getCurrentJointValues();
        std::ostringstream joint_values_stream;
        for (const auto& value : joint_values) {
            joint_values_stream << value << " ";
        }
        RCLCPP_INFO(this->get_logger(), "Current joint values: %s", joint_values_stream.str().c_str());

        // Set joint value
        int joint_index = 0; // there is only one joint in the "gripper" group
        joint_values[joint_index] = PI*angle/180;
        bool success = gripper_move_group->setJointValueTarget(joint_values);   
        if (success) {
            // Plan and execute the motion
            auto execution_success = gripper_move_group->move();
            if (execution_success) {
                RCLCPP_INFO(this->get_logger(), "Motion execution successful!");
            } else {
                RCLCPP_WARN(this->get_logger(), "Motion execution failed.");
            }
        }
        else{
            RCLCPP_WARN(this->get_logger(), "Planning failed. Unable to find a valid plan.");
        }
    }

    void visionGraspCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!arm_move_group) {
            RCLCPP_WARN(this->get_logger(), "MoveGroupInterface not initialized yet.");
            return;
        }

        current_pose = arm_move_group->getCurrentPose();

        // approach: target XY at current EE height
        target_pose.position.x = msg->pose.position.x;
        target_pose.position.y = msg->pose.position.y;
        target_pose.position.z = current_pose.pose.position.z;
        target_pose.orientation = current_pose.pose.orientation;
        pose_target_plan_and_execute(target_pose);

        // grasp: lower to object height
        target_pose.position.z = msg->pose.position.z;
        pose_target_plan_and_execute(target_pose);
    }

    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        for (const auto& transform : msg->transforms)
        {
        //const auto& t = transform.transform.translation;
        //const auto& r = transform.transform.rotation;
        
        if(transform.child_frame_id == "Cube1"){
            cube1_pose.position.x = transform.transform.translation.x;
            cube1_pose.position.y = transform.transform.translation.y;
            cube1_pose.position.z = transform.transform.translation.z;

            cube1_pose.orientation = transform.transform.rotation;
        }
        else{ //Cube2
            cube2_pose.position.x = transform.transform.translation.x;
            cube2_pose.position.y = transform.transform.translation.y;
            cube2_pose.position.z = transform.transform.translation.z;

            cube2_pose.orientation = transform.transform.rotation;
        }
        /*
        RCLCPP_INFO(this->get_logger(),
                    "TF from '%s' to '%s'\n"
                    "  Translation: x=%.3f, y=%.3f, z=%.3f\n"
                    "  Rotation (quaternion): x=%.6f, y=%.6f, z=%.6f, w=%.6f",
                    transform.header.frame_id.c_str(),
                    transform.child_frame_id.c_str(),
                    t.x, t.y, t.z,
                    r.x, r.y, r.z, r.w);
        */
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_grasp_sub_;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_move_group;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_move_group;
    geometry_msgs::msg::PoseStamped current_pose;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Create the node and manage it with a shared pointer
    auto node = std::make_shared<JoyControlRobot>();

    // Initialize MoveGroupInterface after the node is managed by std::shared_ptr
    node->initializeMoveGroup();

    // Spin the node
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
