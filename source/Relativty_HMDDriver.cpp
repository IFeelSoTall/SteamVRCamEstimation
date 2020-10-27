// Copyright (C) 2020  Max Coutte, Gabriel Combe
// Copyright (C) 2020  Relativty.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma comment(lib, "Ws2_32.lib")
#pragma comment (lib, "Setupapi.lib")
#pragma comment(lib, "User32.lib")

#include <atomic>
#include <WinSock2.h>
#include <Windows.h>
#include "hidapi/hidapi.h"
#include "openvr_driver.h"

#include "driverlog.h"

#include "Relativty_HMDDriver.hpp"
#include "Relativty_ServerDriver.hpp"
#include "Relativty_EmbeddedPython.h"
#include "Relativty_components.h"
#include "Relativty_base_device.h"
#include <KalmanFilter.h>

#include <string>

inline void Normalize(float norma[3], float v[3], float max[3], float min[3], int up, int down, float scale[3], float offset[3]) {
	for (int i = 0; i < 4; i++) {
		norma[i] = (((up - down) * ((v[i] - min[i]) / (max[i] - min[i])) + down) / scale[i])+ offset[i];
	}
}

vr::EVRInitError Relativty::HMDDriver::Activate(uint32_t unObjectId) {
	RelativtyDevice::Activate(unObjectId);
	this->setProperties();

	DriverLog("KOKOKOK %s:%d\n", __FUNCTION__, __LINE__);
	
	this->update_pose_thread_worker = std::thread(&Relativty::HMDDriver::update_pose_threaded, this);

	return vr::VRInitError_None;
}

void Relativty::HMDDriver::Deactivate() {
	//this->retrieve_quaternion_isOn = false;
	//this->retrieve_quaternion_thread_worker.join();
	//hid_close(this->handle);
	//hid_exit();

	RelativtyDevice::Deactivate();
	this->update_pose_thread_worker.join();

	Relativty::ServerDriver::Log("Thread0: all threads exit correctly \n");
}
KalmanFilter filtered[3] = { KalmanFilter(2, 15, 1, 1), KalmanFilter(2, 15, 1, 1), KalmanFilter(2, 15, 1, 1) };

void Relativty::HMDDriver::update_pose_threaded() {
	Relativty::ServerDriver::Log("Thread2: successfully started\n");
	filtered[0].SetState(0, 0.1);
	filtered[1].SetState(0, 0.1);
	filtered[2].SetState(0, 0.1);
	while (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid) {
		if (this->new_vector_avaiable) {
			filtered[0].Correct(this->vector_xyz[0]);
			filtered[1].Correct(this->vector_xyz[1]);
			filtered[2].Correct(this->vector_xyz[2]);
			m_Pose.vecPosition[0] = filtered[0].State;
			m_Pose.vecPosition[1] = filtered[1].State;
			m_Pose.vecPosition[2] = filtered[2].State;

			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, m_Pose, sizeof(vr::DriverPose_t));
			this->new_vector_avaiable = false;

		}
	}
	Relativty::ServerDriver::Log("Thread2: successfully stopped\n");
}

Relativty::HMDDriver::HMDDriver(std::string myserial, tracker_Type_e t) :RelativtyDevice(myserial, "akira_") {
	Relativty::HMDDriver::HMDDriver(myserial);
	trackerType = t;
}

Relativty::HMDDriver::HMDDriver(std::string myserial):RelativtyDevice(myserial, "akira_") {
	// keys for use with the settings API
	static const char* const Relativty_hmd_section = "Relativty_hmd";
	DriverLog("KOKOKOK %s:%d\n", __FUNCTION__, __LINE__);
	// openvr api stuff
	m_sRenderModelPath = "{Relativty}/rendermodels/generic_hmd";
	m_sBindPath = "{Relativty}/input/relativty_hmd_profile.json";

	// not openvr api stuff
	Relativty::ServerDriver::Log("Loading Settings\n");
	this->IPD = vr::VRSettings()->GetFloat(Relativty_hmd_section, "IPDmeters");
	this->SecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat(Relativty_hmd_section, "secondsFromVsyncToPhotons");

	this->upperBound = vr::VRSettings()->GetFloat(Relativty_hmd_section, "upperBound");
	this->lowerBound = vr::VRSettings()->GetFloat(Relativty_hmd_section, "lowerBound");
	this->normalizeMinX = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMinX");
	this->normalizeMinY = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMinY");
	this->normalizeMinZ = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMinZ");
	this->normalizeMaxX = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMaxX");
	this->normalizeMaxY = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMaxY");
	this->normalizeMaxZ = vr::VRSettings()->GetFloat(Relativty_hmd_section, "normalizeMaxZ");
	this->scalesCoordinateMeterX = vr::VRSettings()->GetFloat(Relativty_hmd_section, "scalesCoordinateMeterX");
	this->scalesCoordinateMeterY = vr::VRSettings()->GetFloat(Relativty_hmd_section, "scalesCoordinateMeterY");
	this->scalesCoordinateMeterZ = vr::VRSettings()->GetFloat(Relativty_hmd_section, "scalesCoordinateMeterZ");
	this->offsetCoordinateX = vr::VRSettings()->GetFloat(Relativty_hmd_section, "offsetCoordinateX");
	this->offsetCoordinateY = vr::VRSettings()->GetFloat(Relativty_hmd_section, "offsetCoordinateY");
	this->offsetCoordinateZ = vr::VRSettings()->GetFloat(Relativty_hmd_section, "offsetCoordinateZ");
	DriverLog("KOKOKOK %s:%d\n", __FUNCTION__, __LINE__);
	this->m_iPid = vr::VRSettings()->GetInt32(Relativty_hmd_section, "hmdPid");
	this->m_iVid = vr::VRSettings()->GetInt32(Relativty_hmd_section, "hmdVid");

	this->m_bIMUpktIsDMP = vr::VRSettings()->GetBool(Relativty_hmd_section, "hmdIMUdmpPackets");


	DriverLog("KOKOKOK %s:%d\n", __FUNCTION__, __LINE__);
	// this is a bad idea, this should be set by the tracking loop
	m_Pose.result = vr::TrackingResult_Running_OK;

}

inline void Relativty::HMDDriver::setProperties() {
	//vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_UserIpdMeters_Float, this->IPD);
	//vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_UserHeadToEyeDepthMeters_Float, 0.16f);
	//vr::VRProperties()->SetFloatProperty(m_ulPropertyContainer, vr::Prop_SecondsFromVsyncToPhotons_Float, this->SecondsFromVsyncToPhotons);

	// avoid "not fullscreen" warnings from vrmonitor
	//vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, vr::Prop_IsOnDesktop_Bool, false);
}
