// vive_node: publishes HMD/controller/tracker poses and controller joystick state
// using OpenVR. Originally written for ROS 1; ported to ROS 2 (Humble).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <std_srvs/srv/empty.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "vive_ros/vr_interface.h"

namespace vive_ros
{

class VIVENode : public rclcpp::Node
{
public:
  explicit VIVENode(int rate)
  : rclcpp::Node("vive_node"),
    rate_hz_(rate)
  {
    world_offset_ = this->declare_parameter<std::vector<double>>(
      "world_offset", std::vector<double>{0.0, 0.0, 0.0});
    world_yaw_ = this->declare_parameter<double>("world_yaw", 0.0);

    if (world_offset_.size() != 3) {
      RCLCPP_WARN(get_logger(), "world_offset must have 3 elements; resetting to zeros.");
      world_offset_ = {0.0, 0.0, 0.0};
    }

    RCLCPP_INFO(
      get_logger(), " [VIVE] World offset: [%2.3f , %2.3f, %2.3f] %2.3f",
      world_offset_[0], world_offset_[1], world_offset_[2], world_yaw_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    set_origin_server_ = this->create_service<std_srvs::srv::Empty>(
      "/vive/set_origin",
      std::bind(&VIVENode::setOriginCB, this, std::placeholders::_1, std::placeholders::_2));

    feedback_sub_ = this->create_subscription<sensor_msgs::msg::JoyFeedback>(
      "/vive/set_feedback", 10,
      std::bind(&VIVENode::set_feedback, this, std::placeholders::_1));

    hmd_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/vive/hmd_pose", 1);
    controller1_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/vive/controller1_pose", 1);
    controller2_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/vive/controller2_pose", 1);
  }

  bool Init()
  {
    vr_.setDebugMsgCallback(
      [this](const std::string & m) {RCLCPP_DEBUG(get_logger(), " [VIVE] %s", m.c_str());});
    vr_.setInfoMsgCallback(
      [this](const std::string & m) {RCLCPP_INFO(get_logger(), " [VIVE] %s", m.c_str());});
    vr_.setErrorMsgCallback(
      [this](const std::string & m) {RCLCPP_ERROR(get_logger(), " [VIVE] %s", m.c_str());});

    return vr_.Init();
  }

  void Shutdown()
  {
    vr_.Shutdown();
  }

  void Run()
  {
    rclcpp::Rate loop_rate(rate_hz_);
    int run_hz_count = 0;
    auto last_log = now();

    while (rclcpp::ok()) {
      vr_.Update();

      double tf_matrix[3][4];
      for (int i = 0; i < static_cast<int>(vr::k_unMaxTrackedDeviceCount); ++i) {
        int dev_type = vr_.GetDeviceMatrix(i, tf_matrix);
        if (dev_type == 0) {
          continue;
        }

        const rclcpp::Time stamp = now();

        geometry_msgs::msg::PoseStamped msg;
        msg.header.frame_id = "world_vive";
        msg.header.stamp = stamp;
        msg.pose.position.x = tf_matrix[0][3];
        msg.pose.position.y = tf_matrix[1][3];
        msg.pose.position.z = tf_matrix[2][3];

        tf2::Matrix3x3 rot_matrix(
          tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
          tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
          tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);
        tf2::Quaternion quat;
        rot_matrix.getRotation(quat);

        msg.pose.orientation.x = quat.x();
        msg.pose.orientation.y = quat.y();
        msg.pose.orientation.z = quat.z();
        msg.pose.orientation.w = quat.w();

        std::string cur_sn = vr_.GetTrackedDeviceString(
          vr_.pHMD_, i, vr::Prop_SerialNumber_String);
        std::replace(cur_sn.begin(), cur_sn.end(), '-', '_');

        geometry_msgs::msg::TransformStamped device_tf;
        device_tf.header.stamp = stamp;
        device_tf.header.frame_id = "world_vive";
        device_tf.transform.translation.x = tf_matrix[0][3];
        device_tf.transform.translation.y = tf_matrix[1][3];
        device_tf.transform.translation.z = tf_matrix[2][3];
        device_tf.transform.rotation = msg.pose.orientation;

        switch (dev_type) {
          case 1:  // HMD
            device_tf.child_frame_id = "hmd";
            tf_broadcaster_->sendTransform(device_tf);
            hmd_pub_->publish(msg);
            break;

          case 2: {  // Controller
            const std::string controller_frame = "controller_" + cur_sn;
            device_tf.child_frame_id = controller_frame;
            tf_broadcaster_->sendTransform(device_tf);
            publishControllerJoy(i, cur_sn, stamp);
            controller1_pub_->publish(msg);
            break;
          }

          case 3:  // Tracker
            device_tf.child_frame_id = "tracker_" + cur_sn;
            tf_broadcaster_->sendTransform(device_tf);
            break;

          case 4:  // Lighthouse
            device_tf.child_frame_id = "lighthouse_" + cur_sn;
            tf_broadcaster_->sendTransform(device_tf);
            break;

          default:
            break;
        }
      }

      // Publish corrective transform world -> world_vive
      geometry_msgs::msg::TransformStamped world_tf;
      world_tf.header.stamp = now();
      world_tf.header.frame_id = "world";
      world_tf.child_frame_id = "world_vive";
      world_tf.transform.translation.x = world_offset_[0];
      world_tf.transform.translation.y = world_offset_[1];
      world_tf.transform.translation.z = world_offset_[2];
      tf2::Quaternion quat_world;
      quat_world.setRPY(M_PI / 2.0, 0.0, world_yaw_);
      world_tf.transform.rotation.x = quat_world.x();
      world_tf.transform.rotation.y = quat_world.y();
      world_tf.transform.rotation.z = quat_world.z();
      world_tf.transform.rotation.w = quat_world.w();
      tf_broadcaster_->sendTransform(world_tf);

      ++run_hz_count;
      auto current = now();
      if ((current - last_log).seconds() >= 1.0) {
        RCLCPP_INFO(get_logger(), "Run() @ %d [fps]", run_hz_count);
        run_hz_count = 0;
        last_log = current;
      }

      rclcpp::spin_some(this->get_node_base_interface());
      loop_rate.sleep();
    }
  }

private:
  void setOriginCB(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    double tf_matrix[3][4];
    int index = 1;
    int dev_type = 0;
    while (dev_type != 2 && index < static_cast<int>(vr::k_unMaxTrackedDeviceCount)) {
      dev_type = vr_.GetDeviceMatrix(index++, tf_matrix);
    }
    if (dev_type != 2) {
      RCLCPP_WARN(get_logger(), " [VIVE] Couldn't find controller 1.");
      return;
    }

    tf2::Matrix3x3 rot_matrix(
      tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
      tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
      tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);

    tf2::Vector3 c_z = rot_matrix * tf2::Vector3(0, 0, 1);
    c_z[1] = 0;
    c_z.normalize();
    double new_yaw = std::acos(tf2::Vector3(0, 0, 1).dot(c_z)) + M_PI_2;
    if (c_z[0] < 0) {
      new_yaw = -new_yaw;
    }
    world_yaw_ = -new_yaw;

    tf2::Matrix3x3 new_rot;
    new_rot.setRPY(0, 0, world_yaw_);
    tf2::Vector3 new_offset =
      new_rot * tf2::Vector3(-tf_matrix[0][3], tf_matrix[2][3], -tf_matrix[1][3]);
    world_offset_ = {new_offset[0], new_offset[1], new_offset[2]};

    this->set_parameter(rclcpp::Parameter("world_offset", world_offset_));
    this->set_parameter(rclcpp::Parameter("world_yaw", world_yaw_));
    RCLCPP_INFO(
      get_logger(), " [VIVE] New world offset: [%2.3f , %2.3f, %2.3f] %2.3f",
      world_offset_[0], world_offset_[1], world_offset_[2], world_yaw_);
  }

  void set_feedback(sensor_msgs::msg::JoyFeedback::ConstSharedPtr msg)
  {
    if (msg->type == sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE) {
      vr_.TriggerHapticPulse(msg->id, 0, static_cast<int>(msg->intensity));
      for (int i = 0; i < 16; ++i) {
        vr_.TriggerHapticPulse(i, 0, static_cast<int>(msg->intensity));
      }
    }
  }

  void publishControllerJoy(int device_index, const std::string & sn, const rclcpp::Time & stamp)
  {
    vr::VRControllerState_t state;
    vr_.HandleInput(device_index, state);
    sensor_msgs::msg::Joy joy;
    joy.header.stamp = stamp;
    joy.header.frame_id = "controller_" + sn;
    joy.buttons.assign(BUTTON_NUM, 0);
    joy.axes.assign(AXES_NUM, 0.0f);
    if ((1ULL << vr::k_EButton_ApplicationMenu) & state.ulButtonPressed) {joy.buttons[0] = 1;}
    if ((1ULL << vr::k_EButton_SteamVR_Trigger) & state.ulButtonPressed) {joy.buttons[1] = 1;}
    if ((1ULL << vr::k_EButton_SteamVR_Touchpad) & state.ulButtonPressed) {joy.buttons[2] = 1;}
    if ((1ULL << vr::k_EButton_Grip) & state.ulButtonPressed) {joy.buttons[3] = 1;}
    joy.axes[0] = state.rAxis[0].x;
    joy.axes[1] = state.rAxis[0].y;
    joy.axes[2] = state.rAxis[1].x;

    auto it = button_states_pubs_map_.find(sn);
    if (it == button_states_pubs_map_.end()) {
      auto pub = create_publisher<sensor_msgs::msg::Joy>(
        "/vive/controller_" + sn + "/joy", 10);
      it = button_states_pubs_map_.emplace(sn, pub).first;
    }
    it->second->publish(joy);
  }

  VRInterface vr_;
  int rate_hz_;

  std::vector<double> world_offset_;
  double world_yaw_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr set_origin_server_;
  rclcpp::Subscription<sensor_msgs::msg::JoyFeedback>::SharedPtr feedback_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr hmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr controller1_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr controller2_pub_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr>
    button_states_pubs_map_;
};

}  // namespace vive_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<vive_ros::VIVENode>(30);
  if (!node->Init()) {
    node->Shutdown();
    rclcpp::shutdown();
    return 1;
  }
  node->Run();
  node->Shutdown();
  rclcpp::shutdown();
  return 0;
}
