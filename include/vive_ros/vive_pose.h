#ifndef VIVE_ROS__VIVE_POSE_H_
#define VIVE_ROS__VIVE_POSE_H_

#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <openvr.h>

namespace vive_ros
{

enum CONTROLLER_INDEX
{
  VRC_LEFT = 0,
  VRC_RIGHT = 1
};

vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix);
vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix);
geometry_msgs::msg::Pose Remap(geometry_msgs::msg::Pose pose);

struct VRControllerButtons
{
  float trigger {0.0f};
  int menu {0};
  int grip {0};
  std::vector<float> touchpad {0.0f, 0.0f};
};

struct VRPose
{
  geometry_msgs::msg::PoseStamped msg;
  geometry_msgs::msg::Pose basePose;
};

struct VRController
{
  bool status {false};
  VRPose pose;
  VRControllerButtons buttons;

  void SetBasePose(const geometry_msgs::msg::Pose & p) {pose.basePose = p;}
};

class VRControlHandler
{
public:
  VRControlHandler()
  {
    pController[VRC_LEFT] = &left;
    pController[VRC_RIGHT] = &right;
  }

  VRController left;
  VRController right;
  VRController * pController[2];
  const char * name[2] = {"Left", "Right"};

  void DebugPrint();
};

}  // namespace vive_ros

#endif  // VIVE_ROS__VIVE_POSE_H_
