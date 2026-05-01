#ifndef VIVE_ROS__VR_INTERFACE_H_
#define VIVE_ROS__VR_INTERFACE_H_

#include <cstdint>
#include <functional>
#include <string>

#include <openvr.h>

namespace vive_ros
{

using DebugMsgCallback = std::function<void(const std::string &)>;
using InfoMsgCallback = std::function<void(const std::string &)>;
using ErrorMsgCallback = std::function<void(const std::string &)>;

constexpr int BUTTON_NUM = 4;
constexpr int AXES_NUM = 3;

class VRInterface
{
public:
  VRInterface();
  ~VRInterface();

  /// Initialise SteamVR. `app_key` is required by recent SteamVR versions
  /// (otherwise VR_Init returns Init_Internal "Unable to init path manager"
  /// for any non-Steam-launched process). It is passed to vrclient.so via the
  /// VR_Init pStartupInfo JSON. `manifest_path`, when non-empty, is registered
  /// (temporarily) with SteamVR before VR_Init so that the app key is known.
  bool Init(const std::string & app_key = "vive_ros.vive_node",
    const std::string & manifest_path = "");
  void Shutdown();

  void Update();
  void UpdateCalibration();
  void HandleInput(vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t & state);
  void TriggerHapticPulse(vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId,
    int usDurationMicroSec);

  int GetDeviceMatrix(int index, double pMatrix[3][4]);
  int GetDeviceVel(int index, double lin_vel[3], double ang_vel[3]);
  bool IsDeviceConnected(int index);

  void setErrorMsgCallback(ErrorMsgCallback fn);
  void setInfoMsgCallback(InfoMsgCallback fn);
  void setDebugMsgCallback(DebugMsgCallback fn);

  std::string GetTrackedDeviceString(vr::IVRSystem * pHmd, vr::TrackedDeviceIndex_t unDevice,
    vr::TrackedDeviceProperty prop, vr::TrackedPropertyError * peError = nullptr);

  vr::IVRSystem * pHMD_ {nullptr};
  vr::TrackedDevicePose_t device_poses_[vr::k_unMaxTrackedDeviceCount];

private:
  vr::IVRChaperone * pChaperone_ {nullptr};
  unsigned int max_devices_;

  DebugMsgCallback debug_;
  InfoMsgCallback info_;
  ErrorMsgCallback error_;

  vr::ChaperoneCalibrationState cal_state_;
  float play_area_[2];
  vr::HmdQuad_t play_quat_;
};

}  // namespace vive_ros

#endif  // VIVE_ROS__VR_INTERFACE_H_
