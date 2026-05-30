#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <cmath>

namespace gazebo {

class OmniWheelPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_  = model;
    world_  = model->GetWorld();
    ros_node_ = gazebo_ros::Node::Get(sdf);

    // Wheel geometry — must match pub_vel.py values
    L_ = 0.125;   // centre-to-wheel distance (m)
    R_ = 0.03;    // wheel radius (m)

    // Wheel IK angles (same convention as pub_vel.py)
    angles_[0] =  M_PI / 4.0;
    angles_[1] =  M_PI / 4.0 + M_PI / 2.0;
    angles_[2] =  M_PI / 4.0 - M_PI;
    angles_[3] =  M_PI / 4.0 - M_PI / 2.0;

    const char* jnames[4] = {
      "first_wheel_joint", "second_wheel_joint",
      "third_wheel_joint",  "fourth_wheel_joint"
    };
    for (int i = 0; i < 4; ++i) {
      joints_[i] = model->GetJoint(jnames[i]);
      if (!joints_[i])
        RCLCPP_WARN(ros_node_->get_logger(), "Joint %s not found", jnames[i]);
    }

    last_cmd_time_ = world_->SimTime();
    cmd_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_out", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        wz_ = msg->angular.z;
        last_cmd_time_ = world_->SimTime();
      });

    odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_br_    = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

    last_time_ = world_->SimTime();

    update_conn_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&OmniWheelPlugin::OnUpdate, this));

    RCLCPP_INFO(ros_node_->get_logger(), "OmniWheelPlugin loaded");
  }

  void OnUpdate()
  {
    double vx, vy, wz;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      // Stop if no command received for 0.3 s (teleop key released)
      if ((world_->SimTime() - last_cmd_time_).Double() > 0.3) {
        vx_ = 0.0; vy_ = 0.0; wz_ = 0.0;
      }
      vx = vx_; vy = vy_; wz = wz_;
    }

    // --- spin all 4 omni wheels ---
    for (int i = 0; i < 4; ++i) {
      if (!joints_[i]) continue;
      double w = (vx * std::sin(angles_[i]) +
                  vy * std::cos(angles_[i]) +
                  L_ * wz) / R_;
      // fmax must be set each step for ODE to apply the velocity motor
      joints_[i]->SetParam("fmax", 0, 50.0);
      joints_[i]->SetParam("vel",  0, w);
    }

    // --- holonomic body motion (replaces planar_move) ---
    auto pose = model_->WorldPose();
    double yaw = pose.Rot().Yaw();

    // Keep flat
    model_->SetWorldPose(ignition::math::Pose3d(
      pose.Pos(),
      ignition::math::Quaterniond(0, 0, yaw)));

    // Rotate body-frame velocity into world frame
    model_->SetLinearVel(ignition::math::Vector3d(
      vx * std::cos(yaw) - vy * std::sin(yaw),
      vx * std::sin(yaw) + vy * std::cos(yaw),
      0.0));
    model_->SetAngularVel(ignition::math::Vector3d(0, 0, wz));

    // --- odometry ---
    auto now = world_->SimTime();
    if ((now - last_time_).Double() >= 0.05) {   // 20 Hz
      PublishOdom(vx, vy, wz, pose, yaw);
      last_time_ = now;
    }
  }

private:
  void PublishOdom(double vx, double vy, double wz,
                   const ignition::math::Pose3d& pose, double yaw)
  {
    auto stamp = ros_node_->now();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_footprint";

    odom.pose.pose.position.x    = pose.Pos().X();
    odom.pose.pose.position.y    = pose.Pos().Y();
    odom.pose.pose.orientation.z = std::sin(yaw / 2.0);
    odom.pose.pose.orientation.w = std::cos(yaw / 2.0);

    odom.twist.twist.linear.x  = vx;
    odom.twist.twist.linear.y  = vy;
    odom.twist.twist.angular.z = wz;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header           = odom.header;
    tf.child_frame_id   = "base_footprint";
    tf.transform.translation.x = odom.pose.pose.position.x;
    tf.transform.translation.y = odom.pose.pose.position.y;
    tf.transform.rotation      = odom.pose.pose.orientation;
    tf_br_->sendTransform(tf);
  }

  physics::ModelPtr  model_;
  physics::WorldPtr  world_;
  gazebo_ros::Node::SharedPtr ros_node_;
  event::ConnectionPtr update_conn_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>              tf_br_;

  physics::JointPtr joints_[4];
  double angles_[4];
  double L_, R_;

  double vx_ = 0, vy_ = 0, wz_ = 0;
  std::mutex mtx_;
  common::Time last_time_;
  common::Time last_cmd_time_;
};

GZ_REGISTER_MODEL_PLUGIN(OmniWheelPlugin)

} // namespace gazebo
