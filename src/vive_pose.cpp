// vive_pose: standalone publisher of left/right controller poses + buttons.
// Originally written by Jonathan Osterberg; ported to ROS 2 (Humble).

#include "vive_ros/vive_pose.h"
#include "vive_ros/vr_interface.h"

#include <bitset>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>

#include <openvr.h>

namespace vive_ros
{

geometry_msgs::msg::Pose Remap(geometry_msgs::msg::Pose pose)
{
  const double x = pose.position.x;
  const double y = pose.position.y;
  const double z = pose.position.z;

  pose.position.x = -z;
  pose.position.y = -x;
  pose.position.z = y;

  return pose;
}

void VRControlHandler::DebugPrint()
{
  for (int i = 0; i < 2; ++i) {
    std::printf("\n%-15s%-10s%-10s%-10s%-10s\n", name[i], "x", "y", "z", "w");
    std::printf(
      "%-15s%-10.4f%-10.4f%-10.4f%-10.4f\n", "Pose",
      pController[i]->pose.msg.pose.position.x,
      pController[i]->pose.msg.pose.position.y,
      pController[i]->pose.msg.pose.position.z, 0.0);
    std::printf(
      "%-15s%-10.4f%-10.4f%-10.4f%-10.4f\n", "Orientation",
      pController[i]->pose.msg.pose.orientation.x,
      pController[i]->pose.msg.pose.orientation.y,
      pController[i]->pose.msg.pose.orientation.z,
      pController[i]->pose.msg.pose.orientation.w);
    std::printf("%-15s%-10.4f\n", "Trigger", pController[i]->buttons.trigger);
    std::printf("%-15s%-10i\n", "Status", static_cast<int>(pController[i]->status));
  }
}

vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix)
{
  vr::HmdQuaternion_t q;
  q.w = std::sqrt(std::fmax(0.0, 1.0 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2.0;
  q.x = std::sqrt(std::fmax(0.0, 1.0 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2.0;
  q.y = std::sqrt(std::fmax(0.0, 1.0 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2.0;
  q.z = std::sqrt(std::fmax(0.0, 1.0 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2.0;
  q.x = std::copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
  q.y = std::copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
  q.z = std::copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
  return q;
}

vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix)
{
  vr::HmdVector3_t vec;
  vec.v[0] = matrix.m[0][3];
  vec.v[1] = matrix.m[1][3];
  vec.v[2] = matrix.m[2][3];
  return vec;
}

geometry_msgs::msg::PoseStamped MakeGeometryMsgFromData(
  const vr::HmdVector3_t & controller_position,
  const vr::HmdQuaternion_t & controller_orientation,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  geometry_msgs::msg::PoseStamped controller_msg;
  controller_msg.header.frame_id = frame_id;
  controller_msg.header.stamp = stamp;
  controller_msg.pose.position.x = controller_position.v[0];
  controller_msg.pose.position.y = controller_position.v[1];
  controller_msg.pose.position.z = controller_position.v[2];

  controller_msg.pose = Remap(controller_msg.pose);

  controller_msg.pose.orientation.w = controller_orientation.w;
  controller_msg.pose.orientation.x = -controller_orientation.z;
  controller_msg.pose.orientation.y = -controller_orientation.x;
  controller_msg.pose.orientation.z = controller_orientation.y;

  return controller_msg;
}

void GetControllerState(
  vr::IVRSystem * vr_pointer,
  VRControlHandler & controlHandler,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  for (unsigned int deviceId = 0; deviceId < vr::k_unMaxTrackedDeviceCount; ++deviceId) {
    vr::TrackedDevicePose_t trackedDevicePose;
    vr::VRControllerState_t controllerState;

    if (!vr_pointer->IsTrackedDeviceConnected(deviceId)) {
      continue;
    }

    vr::ETrackedDeviceClass cl = vr_pointer->GetTrackedDeviceClass(deviceId);
    if (cl != vr::ETrackedDeviceClass::TrackedDeviceClass_Controller) {
      continue;
    }

    vr_pointer->GetControllerStateWithPose(
      vr::TrackingUniverseStanding, deviceId, &controllerState,
      sizeof(controllerState), &trackedDevicePose);

    vr::ETrackedControllerRole role =
      vr_pointer->GetControllerRoleForTrackedDeviceIndex(deviceId);
    vr_pointer->GetControllerState(deviceId, &controllerState, sizeof(controllerState));

    const int controllerIndex =
      (role == vr::TrackedControllerRole_LeftHand) ? VRC_LEFT : VRC_RIGHT;

    auto * c = controlHandler.pController[controllerIndex];
    c->buttons.trigger = controllerState.rAxis[1].x;
    c->buttons.touchpad[0] = controllerState.rAxis[0].x;
    c->buttons.touchpad[1] = controllerState.rAxis[0].y;

    std::bitset<8> digitalButtons(controllerState.ulButtonPressed);
    c->buttons.menu = static_cast<int>(digitalButtons[1]);
    c->buttons.grip = static_cast<int>(digitalButtons[2]);
    c->status = trackedDevicePose.bPoseIsValid;

    if (trackedDevicePose.bPoseIsValid) {
      vr::HmdVector3_t pos = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
      vr::HmdQuaternion_t orient = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
      c->pose.msg = MakeGeometryMsgFromData(pos, orient, frame_id, stamp);
    }
  }
}

}  // namespace vive_ros

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("vive_publisher");

  // Use VRInterface helper to handle the manifest + pStartupInfo dance that
  // newer SteamVR requires (avoids "Unable to init path manager:
  // VRInitError_Init_Internal").
  vive_ros::VRInterface vr;
  std::string manifest_path;
  try {
    manifest_path =
      ament_index_cpp::get_package_share_directory("vive_ros") +
      "/manifest/vive_ros.vrmanifest";
  } catch (const std::exception & e) {
    RCLCPP_WARN(node->get_logger(), "Could not locate vive_ros manifest: %s", e.what());
  }
  if (!vr.Init("vive_ros.vive_pose", manifest_path)) {
    RCLCPP_FATAL(node->get_logger(), "VR_Init failed.");
    rclcpp::shutdown();
    return 1;
  }
  vr::IVRSystem * vr_pointer = vr.pHMD_;

  const std::string frame_id =
    node->declare_parameter<std::string>("frame_id", "base");

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_controller_pose[] = {
    node->create_publisher<geometry_msgs::msg::PoseStamped>("/vive/controller/left/pose", 1),
    node->create_publisher<geometry_msgs::msg::PoseStamped>("/vive/controller/right/pose", 1),
  };
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_controller_trigger[] = {
    node->create_publisher<std_msgs::msg::Float32>("/vive/controller/left/trigger", 1),
    node->create_publisher<std_msgs::msg::Float32>("/vive/controller/right/trigger", 1),
  };
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_controller_menu[] = {
    node->create_publisher<std_msgs::msg::Int32>("/vive/controller/left/buttons/menu", 1),
    node->create_publisher<std_msgs::msg::Int32>("/vive/controller/right/buttons/menu", 1),
  };
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_controller_grip[] = {
    node->create_publisher<std_msgs::msg::Int32>("/vive/controller/left/buttons/grip", 1),
    node->create_publisher<std_msgs::msg::Int32>("/vive/controller/right/buttons/grip", 1),
  };
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_controller_touchpad[] = {
    node->create_publisher<std_msgs::msg::Float32MultiArray>("/vive/controller/left/touchpad", 1),
    node->create_publisher<std_msgs::msg::Float32MultiArray>("/vive/controller/right/touchpad", 1),
  };

  vive_ros::VRControlHandler controlHandler;
  RCLCPP_INFO(node->get_logger(), "Started vive_pose node...");

  rclcpp::Rate loop_rate(10.0);
  while (rclcpp::ok()) {
    vive_ros::GetControllerState(vr_pointer, controlHandler, frame_id, node->now());

    for (int i = 0; i < 2; ++i) {
      if (!controlHandler.pController[i]->status) {
        continue;
      }
      auto * c = controlHandler.pController[i];

      pub_controller_pose[i]->publish(c->pose.msg);

      std_msgs::msg::Float32 trigger_msg;
      trigger_msg.data = c->buttons.trigger;
      pub_controller_trigger[i]->publish(trigger_msg);

      std_msgs::msg::Int32 menu_msg;
      menu_msg.data = c->buttons.menu;
      pub_controller_menu[i]->publish(menu_msg);

      std_msgs::msg::Int32 grip_msg;
      grip_msg.data = c->buttons.grip;
      pub_controller_grip[i]->publish(grip_msg);

      std_msgs::msg::Float32MultiArray touchpad_msg;
      touchpad_msg.data = c->buttons.touchpad;
      pub_controller_touchpad[i]->publish(touchpad_msg);
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  vr.Shutdown();
  rclcpp::shutdown();
  return 0;
}
