/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "VehicleIMU.hpp"

#include <px4_platform_common/log.h>

#include <float.h>

using namespace matrix;
using namespace time_literals;

namespace sensors
{

VehicleIMU::VehicleIMU(uint8_t accel_index, uint8_t gyro_index) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::navigation_and_controllers),
	_sensor_accel_sub(this, ORB_ID(sensor_accel), accel_index),
	_sensor_gyro_sub(this, ORB_ID(sensor_gyro), gyro_index),
	_accel_corrections(this, SensorCorrections::SensorType::Accelerometer),
	_gyro_corrections(this, SensorCorrections::SensorType::Gyroscope)
{
	const float configured_interval_us = 1e6f / _param_imu_integ_rate.get();
	_accel_integrator.set_reset_interval(configured_interval_us);
	_gyro_integrator.set_reset_interval(configured_interval_us);
}

VehicleIMU::~VehicleIMU()
{
	Stop();
	perf_free(_publish_interval_perf);
	perf_free(_accel_update_perf);
	perf_free(_gyro_update_perf);
}

bool VehicleIMU::Start()
{
	// force initial updates
	ParametersUpdate(true);

	return _sensor_accel_sub.registerCallback() && _sensor_gyro_sub.registerCallback();
}

void VehicleIMU::Stop()
{
	// clear all registered callbacks
	_sensor_accel_sub.unregisterCallback();
	_sensor_gyro_sub.unregisterCallback();

	Deinit();
}

void VehicleIMU::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		_accel_corrections.ParametersUpdate();
		_gyro_corrections.ParametersUpdate();

		// force update
		_accel_interval.update_interval = 0;
		_gyro_interval.update_interval = 0;

		// constrain IMU integration time 1-20 milliseconds (50-1000 Hz)
		int32_t imu_integration_rate_hz = math::constrain(_param_imu_integ_rate.get(), 50, 1000);

		if (imu_integration_rate_hz != _param_imu_integ_rate.get()) {
			_param_imu_integ_rate.set(imu_integration_rate_hz);
			_param_imu_integ_rate.commit_no_notification();
		}
	}
}

bool VehicleIMU::UpdateIntervalAverage(IntervalAverage &intavg, const hrt_abstime &timestamp_sample)
{
	bool updated = false;

	if ((intavg.timestamp_sample_last > 0) && (timestamp_sample > intavg.timestamp_sample_last)) {
		intavg.interval_sum += (timestamp_sample - intavg.timestamp_sample_last);
		intavg.interval_count++;

	} else {
		intavg.interval_sum = 0.f;
		intavg.interval_count = 0.f;
	}

	intavg.timestamp_sample_last = timestamp_sample;

	// periodically calculate sensor update rate
	if (intavg.interval_count > 10000 || ((intavg.update_interval <= FLT_EPSILON) && intavg.interval_count > 100)) {

		const float sample_interval_avg = intavg.interval_sum / intavg.interval_count;

		if (PX4_ISFINITE(sample_interval_avg) && (sample_interval_avg > 0.f)) {
			// update if interval has changed by more than 1%
			if ((fabsf(intavg.update_interval - sample_interval_avg) / intavg.update_interval) > 0.01f) {

				intavg.update_interval = sample_interval_avg;
				updated = true;
			}
		}

		// reset sample interval accumulator
		intavg.timestamp_sample_last = 0;
	}

	return updated;
}

void VehicleIMU::Run()
{
	ParametersUpdate();
	_accel_corrections.SensorCorrectionsUpdate();
	_gyro_corrections.SensorCorrectionsUpdate();

	while (_sensor_accel_sub.updated() || _sensor_gyro_sub.updated()) {

		bool update_integrator_config = false;

		// update accel
		sensor_accel_s accel;

		if (_sensor_accel_sub.update(&accel)) {
			perf_count_interval(_accel_update_perf, accel.timestamp_sample);
			_accel_corrections.set_device_id(accel.device_id);

			const Vector3f accel_corrected{_accel_corrections.Correct(Vector3f{accel.x, accel.y, accel.z})};
			_accel_integrator.put(accel.timestamp_sample, accel_corrected);
			_last_timestamp_sample_accel = accel.timestamp_sample;

			// collect sample interval average for filters
			if (UpdateIntervalAverage(_accel_interval, accel.timestamp_sample)) {
				update_integrator_config = true;
			}
		}


		// update gyro
		sensor_gyro_s gyro;

		if (_sensor_gyro_sub.update(&gyro)) {
			perf_count_interval(_gyro_update_perf, gyro.timestamp_sample);
			_gyro_corrections.set_device_id(gyro.device_id);

			const Vector3f gyro_corrected{_gyro_corrections.Correct(Vector3f{gyro.x, gyro.y, gyro.z})};
			_gyro_integrator.put(gyro.timestamp_sample, gyro_corrected);
			_last_timestamp_sample_gyro = gyro.timestamp_sample;

			// collect sample interval average for filters
			if (UpdateIntervalAverage(_gyro_interval, gyro.timestamp_sample)) {
				update_integrator_config = true;
			}
		}

		// reconfigure integrators if calculated sensor intervals have changed
		if (update_integrator_config) {
			UpdateIntergratorConfiguration();
		}

		// publish if both accel & gyro integrators are ready
		if (_accel_integrator.integral_ready() && _gyro_integrator.integral_ready()) {

			uint32_t accel_integral_dt;
			uint32_t gyro_integral_dt;
			Vector3f delta_angle;
			Vector3f delta_velocity;

			if (_accel_integrator.reset(delta_velocity, accel_integral_dt)
			    && _gyro_integrator.reset(delta_angle, gyro_integral_dt)) {

				// publich vehicle_imu
				vehicle_imu_s imu{};
				imu.timestamp_sample = (_last_timestamp_sample_accel + _last_timestamp_sample_gyro) / 2;
				imu.accel_device_id = _accel_corrections.get_device_id();
				imu.gyro_device_id = _gyro_corrections.get_device_id();

				delta_angle.copyTo(imu.delta_angle);
				delta_velocity.copyTo(imu.delta_velocity);

				imu.dt = (accel_integral_dt + gyro_integral_dt) / 2;
				imu.dt_accel = accel_integral_dt;
				imu.dt_gyro = gyro_integral_dt;
				//imu.clip_count = accel.clip_count;
				imu.timestamp = hrt_absolute_time();

				_vehicle_imu_pub.publish(imu);

				perf_count_interval(_publish_interval_perf, imu.timestamp);

				return;
			}
		}
	}
}

void VehicleIMU::UpdateIntergratorConfiguration()
{
	const float configured_interval_us = 1e6f / _param_imu_integ_rate.get();

	if ((_accel_interval.update_interval > 0) && (_gyro_interval.update_interval > 0)) {

		const float integral_samples = roundf(configured_interval_us / _gyro_interval.update_interval);
		const float interval_us = roundf(integral_samples * _gyro_interval.update_interval);

		// relaxed minimum reset interval (closest we can get to configured interval minus half a sample interval to tolerate any jitter)
		const float relaxed_reset_interval_min = roundf(interval_us - 0.5f * _gyro_interval.update_interval);

		_gyro_integrator.set_reset_samples(integral_samples);
		_gyro_integrator.set_reset_interval(relaxed_reset_interval_min);

		_sensor_accel_sub.set_required_updates(integral_samples);
		_sensor_gyro_sub.set_required_updates(integral_samples);

		if ((_accel_corrections.get_device_id() == _gyro_corrections.get_device_id())
		    && fabsf(_accel_interval.update_interval - _gyro_interval.update_interval) < 10) {

			// if accel and gyro are coming form the same device and data rates are extremely close we can schedule as an integer multipe of samples
			_accel_integrator.set_reset_samples(integral_samples);
			_accel_integrator.set_reset_interval(relaxed_reset_interval_min);

		} else {
			// otherwise let the gyro set the configuration and scheduling
			// set conservative minimum for accel integrator reset (it will be forced to reset when gyro integrator is ready)
			_accel_integrator.set_reset_samples(1);
			_accel_integrator.set_reset_interval(0);

			// run when there are enough new gyro samples, unregister accel
			_sensor_accel_sub.unregisterCallback();
		}

		PX4_DEBUG("accel (%d) gyro (%d) interval updated: %.0f us, gyro samples: %.0f",
			  _accel_corrections.get_device_id(), _gyro_corrections.get_device_id(), (double)interval_us, (double)integral_samples);
	}
}

void VehicleIMU::PrintStatus()
{
	PX4_INFO("selected IMU: accel: %d gyro: %d ", _accel_corrections.get_device_id(), _gyro_corrections.get_device_id());
	perf_print_counter(_publish_interval_perf);
	perf_print_counter(_accel_update_perf);
	perf_print_counter(_gyro_update_perf);
	_accel_corrections.PrintStatus();
	_gyro_corrections.PrintStatus();
}

} // namespace sensors
