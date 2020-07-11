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

#include "VehicleOpticalFlow.hpp"

#include <px4_platform_common/log.h>

namespace sensors
{

using namespace matrix;
using namespace time_literals;

static constexpr uint32_t SENSOR_TIMEOUT{300_ms};

VehicleOpticalFlow::VehicleOpticalFlow() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_voter.set_timeout(SENSOR_TIMEOUT);
	_voter.set_equal_value_threshold(1000);

	ParametersUpdate(true);
}

VehicleOpticalFlow::~VehicleOpticalFlow()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleOpticalFlow::Start()
{
	ScheduleNow();
	return true;
}

void VehicleOpticalFlow::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_sub) {
		sub.unregisterCallback();
	}
}

void VehicleOpticalFlow::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		_sensor_gyro_calibration.ParametersUpdate();

		// update priority (SENS_FLOWx_PRIO)
		for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
			const int32_t priority_old = _configuration[i].priority();
			_configuration[i].ParametersUpdate();
			const int32_t priority_new = _configuration[i].priority();

			if (priority_old != priority_new) {
				if (_priority[i] == priority_old) {
					_priority[i] = priority_new;

				} else {
					// change relative priority to incorporate any sensor faults
					int priority_change = priority_new - priority_old;
					_priority[i] = math::constrain(_priority[i] + priority_change, 1, 100);
				}
			}
		}
	}
}

void VehicleOpticalFlow::Run()
{
	perf_begin(_cycle_perf);

	ParametersUpdate();

	if (_sensor_selection_sub.updated()) {
		sensor_selection_s sensor_selection;

		if (_sensor_selection_sub.copy(&sensor_selection)) {
			// find primary change instance
			for (uint8_t gyro_index = 0; gyro_index < 3; gyro_index++) {
				uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID::sensor_gyro, gyro_index};

				if ((sensor_selection.gyro_device_id != 0) && (sensor_gyro_sub.get().device_id == sensor_selection.gyro_device_id)) {
					// each flow sensor has a gyro subscription
					for (int flow_instance = 0; flow_instance < MAX_SENSOR_COUNT; flow_instance++) {
						_sensor_gyro_sub[flow_instance].ChangeInstance(gyro_index);
					}

					_sensor_gyro_calibration.set_device_id(sensor_gyro_sub.get().device_id);
					break;
				}
			}
		}
	}

	bool updated[MAX_SENSOR_COUNT] {};

	for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {
		if (!_advertised[uorb_index]) {
			// use data's timestamp to throttle advertisement checks
			if (hrt_elapsed_time(&_last_data[uorb_index].timestamp) > 1_s) {
				if (_sensor_sub[uorb_index].advertised()) {
					if (uorb_index > 0) {
						/* the first always exists, but for each further sensor, add a new validator */
						if (!_voter.add_new_validator()) {
							PX4_ERR("failed to add validator for %s %i", "FLOW", uorb_index);
						}
					}

					_advertised[uorb_index] = true;

					// advertise outputs in order if publishing all
					if (!_param_sens_flow_mode.get()) {
						for (int instance = 0; instance < uorb_index; instance++) {
							_vehicle_optical_flow_multi_pub[instance].advertise();
						}
					}

				} else {
					_last_data[uorb_index].timestamp = hrt_absolute_time();
				}
			}
		}

		if (_advertised[uorb_index]) {
			while (_sensor_sub[uorb_index].update(&_last_data[uorb_index])) {
				updated[uorb_index] = true;

				if (_configuration[uorb_index].device_id() != _last_data[uorb_index].device_id) {
					_configuration[uorb_index].set_device_id(_last_data[uorb_index].device_id);
					_priority[uorb_index] = _configuration[uorb_index].priority();
				}

				float flow_array[3] {_last_data[uorb_index].pixel_flow_x, _last_data[uorb_index].pixel_flow_y, 0};
				_voter.put(uorb_index, _last_data[uorb_index].timestamp, flow_array, _last_data[uorb_index].error_count,
					   _priority[uorb_index]);

				_timestamp_sample_sum[uorb_index] += _last_data[uorb_index].timestamp_sample;

				_optical_flow_dt[uorb_index] += math::max(_last_data[uorb_index].timestamp_sample -
								_timestamp_sample_previous[uorb_index], 100_ms);
				_timestamp_sample_previous[uorb_index] = _last_data[uorb_index].timestamp_sample;

				_optical_flow_sum[uorb_index] += Vector2f{flow_array};
				_quality_sum[uorb_index] += _last_data[uorb_index].quality;
				_sum_count[uorb_index]++;
			}
		}
	}

	// integrate raw gyro for each flow up to timestamp_sample
	for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {
		if (_advertised[uorb_index] && (_timestamp_sample_previous[uorb_index] != 0)) {
			sensor_gyro_s sensor_gyro{};

			while (_sensor_gyro_sub[uorb_index].update(&sensor_gyro)) {
				const Vector3f gyro{_sensor_gyro_calibration.Correct(Vector3f{sensor_gyro.x, sensor_gyro.y, sensor_gyro.z})};

				_gyro_integrator[uorb_index].put(sensor_gyro.timestamp_sample, gyro);

				if (sensor_gyro.timestamp_sample >= _timestamp_sample_previous[uorb_index]) {
					break;
				}
			}
		}
	}

	// check for the current best sensor
	int best_index = 0;
	_voter.get_best(hrt_absolute_time(), &best_index);

	if (best_index >= 0) {
		if (_selected_sensor_sub_index != best_index) {
			// clear all registered callbacks
			for (auto &sub : _sensor_sub) {
				sub.unregisterCallback();
			}

			if (_param_sens_flow_mode.get()) {
				if (_selected_sensor_sub_index >= 0) {
					PX4_INFO("%s switch from #%u -> #%d", "FLOW", _selected_sensor_sub_index, best_index);
				}
			}

			_selected_sensor_sub_index = best_index;
			_sensor_sub[_selected_sensor_sub_index].registerCallback();
		}
	}

	// Publish
	if (_param_sens_flow_mode.get()) {
		// publish only best sensor
		if ((_selected_sensor_sub_index >= 0)
		    && (_voter.get_sensor_state(_selected_sensor_sub_index) == DataValidator::ERROR_FLAG_NO_ERROR)
		    && updated[_selected_sensor_sub_index]) {

			Publish(_selected_sensor_sub_index);
		}

	} else {
		// publish all
		for (int uorb_index = 0; uorb_index < MAX_SENSOR_COUNT; uorb_index++) {
			// publish all sensors as separate instances
			if (updated[uorb_index] && (_configuration[uorb_index].device_id() != 0)) {
				Publish(uorb_index, true);
			}
		}
	}


	// check failover and report
	if (_param_sens_flow_mode.get()) {
		if (_last_failover_count != _voter.failover_count()) {
			uint32_t flags = _voter.failover_state();
			int failover_index = _voter.failover_index();

			if (flags != DataValidator::ERROR_FLAG_NO_ERROR) {
				if (failover_index != -1) {
					const hrt_abstime now = hrt_absolute_time();

					if (now - _last_error_message > 3_s) {
						mavlink_log_emergency(&_mavlink_log_pub, "%s #%i failed: %s%s%s%s%s!",
								      "FLOW",
								      failover_index,
								      ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " OFF" : ""),
								      ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " STALE" : ""),
								      ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " TIMEOUT" : ""),
								      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " ERR CNT" : ""),
								      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " ERR DNST" : ""));
						_last_error_message = now;
					}

					// reduce priority of failed sensor to the minimum
					_priority[failover_index] = 1;
				}
			}

			_last_failover_count = _voter.failover_count();
		}
	}

	// reschedule timeout
	ScheduleDelayed(5_ms);

	perf_end(_cycle_perf);
}

void VehicleOpticalFlow::Publish(uint8_t instance, bool multi)
{
	if ((_param_sens_flow_rate.get() > 0)
	    && hrt_elapsed_time(&_last_publication_timestamp[instance]) >= (1e6f / _param_sens_flow_rate.get())) {

		const hrt_abstime timestamp_sample = _timestamp_sample_sum[instance] / _sum_count[instance];
		const Vector2f optical_flow_data = _optical_flow_sum[instance] / _sum_count[instance];

		// populate with primary sensor and publish
		vehicle_optical_flow_s out{};
		out.timestamp_sample = timestamp_sample;
		out.device_id = _configuration[instance].device_id();
		out.pixel_flow_x_integral = optical_flow_data(0);
		out.pixel_flow_y_integral = optical_flow_data(1);

		out.integration_timespan = _optical_flow_dt[instance];

		out.gyro_x_rate_integral = NAN;
		out.gyro_y_rate_integral = NAN;
		out.gyro_z_rate_integral = NAN;

		if (_gyro_integrator[instance].integral_ready()) {
			uint32_t gyro_integral_dt;
			Vector3f delta_angle;

			if (_gyro_integrator[instance].reset(delta_angle, gyro_integral_dt)) {

				out.gyro_x_rate_integral = delta_angle(0);
				out.gyro_y_rate_integral = delta_angle(1);
				out.gyro_z_rate_integral = delta_angle(2);

				out.integration_timespan_gyro = gyro_integral_dt;

				//PX4_INFO("gyro_integral_dt: %llu, integration_timespan: %llu", gyro_integral_dt, out.integration_timespan);
			}
		}

		out.timestamp = hrt_absolute_time();

		if (multi) {
			_vehicle_optical_flow_multi_pub[instance].publish(out);

		} else {
			// otherwise only ever publish the first instance
			_vehicle_optical_flow_multi_pub[0].publish(out);
		}

		_last_publication_timestamp[instance] = out.timestamp;

		// reset
		_timestamp_sample_sum[instance] = 0;
		_optical_flow_sum[instance].zero();
		_sum_count[instance] = 0;
		_optical_flow_dt[instance] = 0;
	}
}

void VehicleOpticalFlow::PrintStatus()
{
	if (_selected_sensor_sub_index >= 0) {
		PX4_INFO("selected optical flow: %d (%d)", _last_data[_selected_sensor_sub_index].device_id,
			 _selected_sensor_sub_index);
	}

	_voter.print();

	for (int i = 0; i < MAX_SENSOR_COUNT; i++) {
		if (_advertised[i] && (_priority[i] > 0)) {
			_configuration[i].PrintStatus();
		}
	}
}

}; // namespace sensors
