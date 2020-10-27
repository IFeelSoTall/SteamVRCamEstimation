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

#pragma once
#include <thread>
#include <atomic>
#include <WinSock2.h>
#include "hidapi/hidapi.h"
#include "openvr_driver.h"
#include "Relativty_components.h"
#include "Relativty_base_device.h"

namespace Relativty {
	class HMDDriver : public RelativtyDevice<false>
	{
	public:
		enum tracker_Type_e {
			LEFT_LEG,
			RiGHT_LEG
		};
		HMDDriver(std::string myserial);
		HMDDriver(std::string myserial, tracker_Type_e t);
		~HMDDriver() = default;
		tracker_Type_e trackerType;
		void frameUpdate();
		inline void setProperties();

		// Inherited from RelativtyDevice, to be overridden
		virtual vr::EVRInitError Activate(uint32_t unObjectId);
		virtual void Deactivate();
		std::atomic<float> vector_xyz[3];
		std::atomic<bool> new_vector_avaiable = false;
	private:
		int32_t m_iPid;
		int32_t m_iVid;

		bool m_bIMUpktIsDMP;

		float SecondsFromVsyncToPhotons;
		float IPD;
		float HeadToEyeDepth;

		vr::DriverPose_t lastPose = {0};

		std::atomic<float> qconj[4] = {1, 0, 0, 0};

		float upperBound;
		float lowerBound;

		float normalizeMinX;
		float normalizeMinY;
		float normalizeMinZ;

		float normalizeMaxX;
		float normalizeMaxY;
		float normalizeMaxZ;

		float scalesCoordinateMeterX;
		float scalesCoordinateMeterY;
		float scalesCoordinateMeterZ;

		float offsetCoordinateX;
		float offsetCoordinateY;
		float offsetCoordinateZ;

		std::thread update_pose_thread_worker;
		void update_pose_threaded();
	};
}