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

#include "openvr_driver.h"

#include "driverlog.h"

#include "Relativty_ServerDriver.hpp"
#include "Relativty_HMDDriver.hpp"
#include "Relativty_EmbeddedPython.h"
#define DRIVERLOG_H

vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix) {
	vr::HmdVector3_t vector;

	vector.v[0] = matrix.m[0][3];
	vector.v[1] = matrix.m[1][3];
	vector.v[2] = matrix.m[2][3];

	return vector;
}

inline void Normalize(float norma[3], float v[3], float max[3], float min[3], int up, int down, float scale[3], float offset[3]) {
	for (int i = 0; i < 4; i++) {
		norma[i] = (((up - down) * ((v[i] - min[i]) / (max[i] - min[i])) + down) / scale[i]) + offset[i];
	}
}

void Relativty::ServerDriver::retrieve_client_vector_packet_threaded() {
	WSADATA wsaData;
	struct sockaddr_in server, client;
	int addressLen = sizeof(struct sockaddr_in);
	char receiveBuffer[36];
	int resultReceiveLen;

	float normalize_min[3]{ this->normalizeMinX, this->normalizeMinY, this->normalizeMinZ };
	float normalize_max[3]{ this->normalizeMaxX, this->normalizeMaxY, this->normalizeMaxZ };
	float scales_coordinate_meter[3]{ this->scalesCoordinateMeterX, this->scalesCoordinateMeterY, this->scalesCoordinateMeterZ };
	float offset_coordinate[3] = { this->offsetCoordinateX, this->offsetCoordinateY, this->offsetCoordinateZ };

	float coordinate_body[3]{ 0, 0, 0 };
	float coordinate_leftLeg[3]{ 0, 0, 0 };
	float coordinate_normalized_body[3];
	float coordinate_normalized_leftLeg[3];

	Relativty::ServerDriver::Log("Thread3: Initialising Socket.\n");
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		Relativty::ServerDriver::Log("Thread3: Failed. Error Code: " + WSAGetLastError());
		return;
	}
	Relativty::ServerDriver::Log("Thread3: Socket successfully initialised.\n");

	if ((this->sock = socket(AF_INET, SOCK_STREAM, 0)) == INVALID_SOCKET)
		Relativty::ServerDriver::Log("Thread3: could not create socket: " + WSAGetLastError());
	Relativty::ServerDriver::Log("Thread3: Socket created.\n");

	server.sin_family = AF_INET;
	server.sin_port = htons(50000);
	server.sin_addr.s_addr = INADDR_ANY;

	if (bind(this->sock, (struct sockaddr*) & server, sizeof(server)) == SOCKET_ERROR)
		Relativty::ServerDriver::Log("Thread3: Bind failed with error code: " + WSAGetLastError());
	Relativty::ServerDriver::Log("Thread3: Bind done \n");

	listen(this->sock, 1);

	this->serverNotReady = false;

	while(1)
	{
		Relativty::ServerDriver::Log("Thread3: Waiting for incoming connections...\n");
		sock_receive = accept(this->sock, (struct sockaddr*) & client, &addressLen);
		if (sock_receive == INVALID_SOCKET)
			Relativty::ServerDriver::Log("Thread3: accept failed with error code: " + WSAGetLastError());
		Relativty::ServerDriver::Log("Thread3: Connection accepted");

		Relativty::ServerDriver::Log("Thread3: successfully started\n");
		while (this->retrieve_vector_isOn) {
			resultReceiveLen = recv(sock_receive, receiveBuffer, sizeof(receiveBuffer), NULL);
			if (resultReceiveLen <= 0)
			{
				Relativty::ServerDriver::Log("Client was disconnected\n");
				closesocket(sock_receive);
				sock_receive = NULL;
				break;
			}
				
			if (resultReceiveLen > 0) {
				vr::TrackedDevicePose_t hmd_pose[10];
				vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0, hmd_pose, 10);
				coordinate_body[0] = *(float*)(receiveBuffer);
				coordinate_body[1] = *(float*)(receiveBuffer + 4);
				coordinate_body[2] = *(float*)(receiveBuffer + 8);
				Normalize(coordinate_normalized_body, coordinate_body, normalize_max, normalize_min, this->upperBound, this->lowerBound, scales_coordinate_meter, offset_coordinate);
				coordinate_leftLeg[0] = *(float*)(receiveBuffer + 12 * 1);
				coordinate_leftLeg[1] = *(float*)(receiveBuffer + 12 * 1 + 4);
				coordinate_leftLeg[2] = *(float*)(receiveBuffer + 12 * 1 + 8);
				Normalize(coordinate_normalized_leftLeg, coordinate_leftLeg, normalize_max, normalize_min, this->upperBound, this->lowerBound, scales_coordinate_meter, offset_coordinate);
				if (coordinate_body[0] == coordinate_leftLeg[0] || coordinate_body[1] == coordinate_leftLeg[1] || coordinate_body[2] == coordinate_leftLeg[2])
					continue;
				//this->vector_xyz[0] = coordinate_normalized[1];
				//this->vector_xyz[1] = coordinate_normalized[2];
				//this->vector_xyz[2] = coordinate_normalized[0];
				HMDDriverLeft->vector_xyz[0] =/* hmd_pose[0].mDeviceToAbsoluteTracking.m[0][3] + coordinate_normalized_body[0] -*/ coordinate_normalized_leftLeg[0];
				HMDDriverLeft->vector_xyz[1] = /*hmd_pose[0].mDeviceToAbsoluteTracking.m[1][3] + coordinate_normalized_body[1] -*/ coordinate_normalized_leftLeg[1];
				HMDDriverLeft->vector_xyz[2] = /*hmd_pose[0].mDeviceToAbsoluteTracking.m[2][3] + coordinate_normalized_body[2] -*/ coordinate_normalized_leftLeg[2];
				//DriverLog("X:%f, Y:%f, Z:%f\n", (float)HMDDriverLeft->vector_xyz[0], (float)HMDDriverLeft->vector_xyz[1], (float)HMDDriverLeft->vector_xyz[2]);
				//DriverLog("HMD X:%f, Y:%f, Z:%f\n", (float)hmd_pose[0].mDeviceToAbsoluteTracking.m[0][3], (float)hmd_pose[0].mDeviceToAbsoluteTracking.m[1][3], (float)hmd_pose[0].mDeviceToAbsoluteTracking.m[2][3]);
				//DriverLog("body X:%f, Y:%f, Z:%f\n", (float)coordinate_body[0], (float)coordinate_body[1], (float)coordinate_normalized_body[2]);
				//DriverLog("leftleg X:%f, Y:%f, Z:%f\n", (float)coordinate_leftLeg[0], (float)coordinate_leftLeg[1], (float)coordinate_leftLeg[2]);
				HMDDriverLeft->new_vector_avaiable = true;
			}
		}
		if (!this->retrieve_vector_isOn)
			break;
	}
	closesocket(sock);
	sock = NULL;
	Relativty::ServerDriver::Log("Thread3: successfully stopped\n");
}

vr::EVRInitError Relativty::ServerDriver::Init(vr::IVRDriverContext* DriverContext) {

	vr::EVRInitError eError = vr::InitServerDriverContext(DriverContext);
		if (eError != vr::VRInitError_None) {
			return eError;
	}
	#ifdef DRIVERLOG_H
	InitDriverLog(vr::VRDriverLog());
	DriverLog("Relativty driver version 0.1.1"); // report driver version
	DriverLog("Thread1: hid quaternion packet listener loop");
	DriverLog("Thread2: update driver pose loop");
	DriverLog("Thread3: receive positional data from python loop");
	#endif

	this->Log("Relativty Init successful.\n");
	static const char* const Relativty_hmd_section = "Relativty_hmd";
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
	char buffer[1024];
	vr::VRSettings()->GetString(Relativty_hmd_section, "PyPath", buffer, sizeof(buffer));
	PyPath = buffer;

	this->retrieve_vector_isOn = true;
	this->retrieve_vector_thread_worker = std::thread(&Relativty::ServerDriver::retrieve_client_vector_packet_threaded, this);
	while (this->serverNotReady) {
		// do nothing
	}

	startPythonTrackingClient_worker = std::thread(startPythonTrackingClient_threaded, this->PyPath);

	this->HMDDriverLeft = new Relativty::HMDDriver("zeroL", HMDDriver::LEFT_LEG);
	//this->HMDDriverRight = new Relativty::HMDDriver("zeroR", HMDDriver::RiGHT_LEG);
	vr::VRServerDriverHost()->TrackedDeviceAdded(HMDDriverLeft->GetSerialNumber().c_str(), vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker, this->HMDDriverLeft);
	//vr::VRServerDriverHost()->TrackedDeviceAdded(HMDDriverRight->GetSerialNumber().c_str(), vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker, this->HMDDriverRight);
	return vr::VRInitError_None;
}

void Relativty::ServerDriver::Cleanup() {
	if (HMDDriverLeft != NULL)
	{
		delete this->HMDDriverLeft;
		this->HMDDriverLeft = NULL;
	}
	if (HMDDriverRight != NULL)
	{
		delete this->HMDDriverRight;
		this->HMDDriverRight = NULL;
	}
	this->retrieve_vector_isOn = false;
	this->retrieve_vector_thread_worker.join();
	if (!sock_receive)
		closesocket(sock_receive);
	if (!sock)
		closesocket(sock);
	WSACleanup();
	#ifdef DRIVERLOG_H
	CleanupDriverLog();
	#endif

	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

const char* const* Relativty::ServerDriver::GetInterfaceVersions() {
	return vr::k_InterfaceVersions;
}

void Relativty::ServerDriver::RunFrame() {} // if ur not using it don't populate it with garbage!

bool Relativty::ServerDriver::ShouldBlockStandbyMode() {
	return false;
}

void Relativty::ServerDriver::EnterStandby() {

}

void Relativty::ServerDriver::LeaveStandby() {

}

void Relativty::ServerDriver::Log(std::string log) {
	vr::VRDriverLog()->Log(log.c_str());
}