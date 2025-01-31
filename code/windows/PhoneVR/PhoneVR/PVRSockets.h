#pragma once

#include "Geometry"
#include "openvr_driver.h"

#include "PVRGlobals.h"
#include "PVRSocketUtils.h"
#include "battery.h"
#include "motion_tracker.h"



void PVRStartConnectionListener(std::function<void(std::string ip, PVR_MSG devType)> callback);
void PVRStopConnectionListener();

void PVRStartStreamer(std::string ip, uint16_t width, uint16_t height, std::function<void(std::vector<uint8_t>)> headerCb, std::function<void()> onErrCb);
void PVRProcessFrame(uint64_t hdl, Eigen::Quaternionf quat);
void PVRStopStreamer();

void PVRStartReceiveData(std::string ip, vr::DriverPose_t *pose, uint32_t *objId, battery_deamon *hdm_batery , motion_tracker *hdm_traker_imu);
void PVRStopReceiveData();
