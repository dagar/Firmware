/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "OpticalFlowConfiguration.hpp"

#include "data_validator/DataValidatorGroup.hpp"
#include "integrator/Integrator.hpp"

#include <lib/sensor_calibration/Gyroscope.hpp>
#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_optical_flow.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_optical_flow.h>

namespace sensors
{
class VehicleOpticalFlow : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VehicleOpticalFlow();
	~VehicleOpticalFlow() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	void ParametersUpdate(bool force = false);

	void Publish(uint8_t instance, bool multi = false);

	static constexpr int MAX_SENSOR_COUNT = 4;

	uORB::PublicationMulti<vehicle_optical_flow_s> _vehicle_optical_flow_multi_pub[MAX_SENSOR_COUNT] {
		{ORB_ID(vehicle_optical_flow)},
		{ORB_ID(vehicle_optical_flow)},
		{ORB_ID(vehicle_optical_flow)},
		{ORB_ID(vehicle_optical_flow)},
	};

	uORB::Subscription _params_sub{ORB_ID(parameter_update)};

	uORB::SubscriptionCallbackWorkItem _sensor_sub[MAX_SENSOR_COUNT] {
		{this, ORB_ID(sensor_optical_flow), 0},
		{this, ORB_ID(sensor_optical_flow), 1},
		{this, ORB_ID(sensor_optical_flow), 2},
		{this, ORB_ID(sensor_optical_flow), 3}
	};

	OpticalFlowConfiguration _configuration[MAX_SENSOR_COUNT];

	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};
	uORB::Subscription _sensor_gyro_sub[MAX_SENSOR_COUNT] {
		ORB_ID(sensor_gyro),
		ORB_ID(sensor_gyro),
		ORB_ID(sensor_gyro),
		ORB_ID(sensor_gyro),
	};
	calibration::Gyroscope _sensor_gyro_calibration{};
	Integrator _gyro_integrator[MAX_SENSOR_COUNT] {true, true, true, true};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	hrt_abstime _last_error_message{0};
	orb_advert_t _mavlink_log_pub{nullptr};

	DataValidatorGroup _voter{1};
	unsigned _last_failover_count{0};

	hrt_abstime _timestamp_sample_previous[MAX_SENSOR_COUNT] {};
	uint64_t _timestamp_sample_sum[MAX_SENSOR_COUNT] {};
	uint64_t _optical_flow_dt[MAX_SENSOR_COUNT] {};
	hrt_abstime _last_publication_timestamp[MAX_SENSOR_COUNT] {};
	matrix::Vector2f _optical_flow_sum[MAX_SENSOR_COUNT] {};
	int _quality_sum[MAX_SENSOR_COUNT] {};
	int _sum_count[MAX_SENSOR_COUNT] {};

	sensor_optical_flow_s _last_data[MAX_SENSOR_COUNT] {};
	bool _advertised[MAX_SENSOR_COUNT] {};

	uint8_t _priority[MAX_SENSOR_COUNT] {};

	int8_t _selected_sensor_sub_index{-1};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::SENS_FLOW_MODE>) _param_sens_flow_mode,
		(ParamFloat<px4::params::SENS_FLOW_RATE>) _param_sens_flow_rate,
		(ParamInt<px4::params::SENS_FLOW_ROT>) _param_sens_flow_rot,
		(ParamFloat<px4::params::SENS_FLOW_MINHGT>) _param_sens_flow_minhgt,
		(ParamFloat<px4::params::SENS_FLOW_MAXHGT>) _param_sens_flow_maxhgt,
		(ParamFloat<px4::params::SENS_FLOW_MAXR>) _param_sens_flow_maxr
	)
};
}; // namespace sensors
