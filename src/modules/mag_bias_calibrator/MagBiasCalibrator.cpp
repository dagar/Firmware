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

#include "MagBiasCalibrator.hpp"

using namespace time_literals;
using matrix::Vector3f;
using math::constrain;

namespace mag_bias_calibrator
{

MagBiasCalibrator::MagBiasCalibrator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

MagBiasCalibrator::~MagBiasCalibrator()
{
	ScheduleClear();
	perf_free(_cycle_perf);
}

int MagBiasCalibrator::task_spawn(int argc, char *argv[])
{
	MagBiasCalibrator *obj = new MagBiasCalibrator();

	if (!obj) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(obj);
	_task_id = task_id_is_work_queue;

	/* Schedule a cycle to start things. */
	obj->start();

	return 0;
}

void MagBiasCalibrator::start()
{
	ScheduleOnInterval(20_ms); // 50 Hz
}

void MagBiasCalibrator::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
	}

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
			_calibration[mag_index].ParametersUpdate();

			_timestamp_last_save[mag_index] = pupdate.timestamp;
		}
	}

	perf_begin(_cycle_perf);

	bool calibration_updated = false;


	Vector3f angular_velocity{};

	for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
		sensor_mag_s sensor_mag;

		while (_sensor_mag_subs[mag_index].update(&sensor_mag)) {

			{
				// grab latest angular velocity
				vehicle_angular_velocity_s vehicle_angular_velocity;

				if (_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity)) {
					angular_velocity = Vector3f{vehicle_angular_velocity.xyz};
				}
			}

			// apply existing mag calibration
			_calibration[mag_index].set_device_id(sensor_mag.device_id);
			const Vector3f mag_calibrated = _calibration[mag_index].Correct(Vector3f{sensor_mag.x, sensor_mag.y, sensor_mag.z});

			const float dt = constrain((sensor_mag.timestamp_sample - _timestamp_last_update[mag_index]) * 1e-6f, 0.001f, 0.2f);
			_timestamp_last_update[mag_index] = sensor_mag.timestamp_sample;

			_bias_estimator[mag_index].updateEstimate(angular_velocity, mag_calibrated, dt);

			const Vector3f &bias = _bias_estimator[mag_index].getBias();

			// save the bias to the parameters to apply it given the right circumstances
			const bool longer_than_10_sec = (sensor_mag.timestamp_sample - _timestamp_last_save[mag_index]) > 10_s;
			const bool bias_significant = bias.norm_squared() > (0.01f * 0.01f);

			if (!_armed && bias_significant && longer_than_10_sec) {
				const Vector3f new_offset = _calibration[mag_index].BiasCorrectedSensorOffset(bias);

				_calibration[mag_index].set_offset(new_offset);
				calibration_updated = true;

				_timestamp_last_save[mag_index] = sensor_mag.timestamp_sample;
			}
		}
	}

	if (calibration_updated) {
		for (int mag_index = 0; mag_index < MAX_SENSOR_COUNT; mag_index++) {
			_calibration[mag_index].set_calibration_index(mag_index);
			_calibration[mag_index].ParametersSave();
		}

		param_notify_changes();
	}

	perf_end(_cycle_perf);
}

int MagBiasCalibrator::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Online magnetometer bias calibrator.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mag_bias_calibrator", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int mag_bias_calibrator_main(int argc, char *argv[])
{
	return MagBiasCalibrator::main(argc, argv);
}

} // namespace load_mon
