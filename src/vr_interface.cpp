#include "vive_ros/vr_interface.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <string>

namespace vive_ros
{

namespace
{

void defaultDebugMsgCallback(const std::string & msg)
{
  std::cerr << "VIVE Debug: " << msg << std::endl;
}

void defaultInfoMsgCallback(const std::string & msg)
{
  std::cerr << "VIVE Info: " << msg << std::endl;
}

void defaultErrorMsgCallback(const std::string & msg)
{
  std::cerr << "VIVE Error: " << msg << std::endl;
}

const std::map<vr::ChaperoneCalibrationState, std::string> kChaperoneStrings = {
  {vr::ChaperoneCalibrationState_OK,
    "Chaperone is fully calibrated and working correctly"},
  {vr::ChaperoneCalibrationState_Warning, "Warning"},
  {vr::ChaperoneCalibrationState_Warning_BaseStationMayHaveMoved,
    "A base station thinks that it might have moved"},
  {vr::ChaperoneCalibrationState_Warning_BaseStationRemoved,
    "There are less base stations than when calibrated"},
  {vr::ChaperoneCalibrationState_Warning_SeatedBoundsInvalid,
    "Seated bounds haven't been calibrated for the current tracking center"},
  {vr::ChaperoneCalibrationState_Error, "The UniverseID is invalid"},
  {vr::ChaperoneCalibrationState_Error_BaseStationUninitialized,
    "Tracking center hasn't be calibrated for at least one of the base stations"},
  {vr::ChaperoneCalibrationState_Error_BaseStationConflict,
    "Tracking center is calibrated, but base stations disagree on the tracking space"},
  {vr::ChaperoneCalibrationState_Error_PlayAreaInvalid,
    "Play Area hasn't been calibrated for the current tracking center"},
  {vr::ChaperoneCalibrationState_Error_CollisionBoundsInvalid,
    "Collision Bounds haven't been calibrated for the current tracking center"},
};

}  // namespace

VRInterface::VRInterface()
: max_devices_(vr::k_unMaxTrackedDeviceCount),
  debug_(defaultDebugMsgCallback),
  info_(defaultInfoMsgCallback),
  error_(defaultErrorMsgCallback)
{
  play_area_[0] = -1.0f;
  play_area_[1] = -1.0f;
  for (int i = 0; i < 4; ++i) {
    for (int o = 0; o < 3; ++o) {
      play_quat_.vCorners[i].v[o] = -1.0f;
    }
  }
}

VRInterface::~VRInterface() = default;

bool VRInterface::Init()
{
  vr::EVRInitError eError = vr::VRInitError_None;
  pHMD_ = vr::VR_Init(&eError, vr::VRApplication_Background);

  if (eError != vr::VRInitError_None) {
    pHMD_ = nullptr;
    error_(std::string("VR_Init Failed: ") +
      vr::VR_GetVRInitErrorAsEnglishDescription(eError));
    return false;
  }

  info_("VR_Init Success.");
  UpdateCalibration();
  return true;
}

void VRInterface::Shutdown()
{
  info_("Shutting down.");
  if (pHMD_) {
    vr::VR_Shutdown();
    pHMD_ = nullptr;
  }
}

void VRInterface::Update()
{
  if (pHMD_) {
    pHMD_->GetDeviceToAbsoluteTrackingPose(
      vr::TrackingUniverseRawAndUncalibrated, 0, device_poses_, max_devices_);
  }
}

bool VRInterface::IsDeviceConnected(int index)
{
  if (static_cast<unsigned int>(index) < max_devices_) {
    return pHMD_->IsTrackedDeviceConnected(index);
  }
  return false;
}

int VRInterface::GetDeviceMatrix(int index, double pMatrix[3][4])
{
  if (static_cast<unsigned int>(index) >= max_devices_) {
    return 0;
  }
  const auto & pose = device_poses_[index];
  if (pose.bDeviceIsConnected && pose.bPoseIsValid &&
    pose.eTrackingResult == vr::TrackingResult_Running_OK)
  {
    for (int i = 0; i < 3; ++i) {
      for (int o = 0; o < 4; ++o) {
        pMatrix[i][o] = static_cast<double>(pose.mDeviceToAbsoluteTracking.m[i][o]);
      }
    }
    return pHMD_->GetTrackedDeviceClass(index);
  }
  return 0;
}

int VRInterface::GetDeviceVel(int index, double lin_vel[3], double ang_vel[3])
{
  if (static_cast<unsigned int>(index) >= max_devices_) {
    return 0;
  }
  const auto & pose = device_poses_[index];
  if (pose.bDeviceIsConnected && pose.eTrackingResult == vr::TrackingResult_Running_OK) {
    for (int i = 0; i < 3; ++i) {
      lin_vel[i] = pose.vVelocity.v[i];
      ang_vel[i] = pose.vAngularVelocity.v[i];
    }
    return pHMD_->GetTrackedDeviceClass(index);
  }
  return 0;
}

void VRInterface::UpdateCalibration()
{
  vr::EVRInitError eError = vr::VRInitError_None;
  pChaperone_ = static_cast<vr::IVRChaperone *>(
    vr::VR_GetGenericInterface(vr::IVRChaperone_Version, &eError));
  if (eError != vr::VRInitError_None || !pChaperone_) {
    error_("Could not find chaperone");
    return;
  }
  cal_state_ = pChaperone_->GetCalibrationState();
  auto state_it = kChaperoneStrings.find(cal_state_);
  info_("Calibration state: " +
    (state_it != kChaperoneStrings.end() ? state_it->second : std::string("Unknown")));

  if (pChaperone_->GetPlayAreaSize(&play_area_[0], &play_area_[1])) {
    info_("Play area: " + std::to_string(play_area_[0]) + " x " +
      std::to_string(play_area_[1]));
  } else {
    info_("Empty play area.");
  }

  if (pChaperone_->GetPlayAreaRect(&play_quat_)) {
    for (int i = 0; i < 4; ++i) {
      info_(
        "Corner " + std::to_string(i) +
        " - x: " + std::to_string(play_quat_.vCorners[i].v[0]) +
        ", y: " + std::to_string(play_quat_.vCorners[i].v[1]) +
        ", z: " + std::to_string(play_quat_.vCorners[i].v[2]));
    }
  }
}

std::string VRInterface::GetTrackedDeviceString(
  vr::IVRSystem * pHmd, vr::TrackedDeviceIndex_t unDevice,
  vr::TrackedDeviceProperty prop, vr::TrackedPropertyError * peError)
{
  uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(
    unDevice, prop, nullptr, 0, peError);
  if (unRequiredBufferLen == 0) {
    return "";
  }
  std::string sResult(unRequiredBufferLen, '\0');
  pHmd->GetStringTrackedDeviceProperty(
    unDevice, prop, sResult.data(), unRequiredBufferLen, peError);
  // Remove the trailing null terminator written by the SDK.
  if (!sResult.empty() && sResult.back() == '\0') {
    sResult.pop_back();
  }
  return sResult;
}

void VRInterface::HandleInput(
  vr::TrackedDeviceIndex_t unControllerDeviceIndex, vr::VRControllerState_t & state)
{
  pHMD_->GetControllerState(
    unControllerDeviceIndex, &state, sizeof(vr::VRControllerState_t));
}

void VRInterface::TriggerHapticPulse(
  vr::TrackedDeviceIndex_t unControllerDeviceIndex, uint32_t unAxisId,
  int usDurationMicroSec)
{
  usDurationMicroSec = std::clamp(usDurationMicroSec, 0, 3999);
  pHMD_->TriggerHapticPulse(
    unControllerDeviceIndex, unAxisId, static_cast<unsigned short>(usDurationMicroSec));
}

void VRInterface::setErrorMsgCallback(ErrorMsgCallback fn) {error_ = std::move(fn);}
void VRInterface::setInfoMsgCallback(InfoMsgCallback fn) {info_ = std::move(fn);}
void VRInterface::setDebugMsgCallback(DebugMsgCallback fn) {debug_ = std::move(fn);}

}  // namespace vive_ros
