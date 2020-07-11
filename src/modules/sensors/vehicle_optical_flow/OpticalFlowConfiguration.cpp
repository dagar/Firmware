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

#include "OpticalFlowConfiguration.hpp"

#include <lib/sensor_calibration/Utilities.hpp>

using calibration::ParamPrefix;
using namespace matrix;
using namespace time_literals;

namespace sensors
{

OpticalFlowConfiguration::OpticalFlowConfiguration()
{
	Reset();
}

OpticalFlowConfiguration::OpticalFlowConfiguration(uint32_t device_id)
{
	Reset();
	set_device_id(device_id);
}

void OpticalFlowConfiguration::set_device_id(uint32_t device_id)
{
	if (_device_id != device_id) {
		_device_id = device_id;
		ParametersUpdate();
	}
}

void OpticalFlowConfiguration::ParametersUpdate()
{
	if (_device_id == 0) {
		Reset();
		return;
	}

	_configuration_index = FindConfigurationIndex(ParamPrefix::SENS, SensorString(), _device_id);

	if (_configuration_index >= 0) {

		// CAL_FLOWx_ROT
		int32_t rotation_value = GetConfigurationParam(ParamPrefix::SENS, SensorString(), "ROT", _configuration_index);

		if ((rotation_value >= ROTATION_MAX) || (rotation_value < 0)) {
			PX4_ERR("External %s %d (%d) invalid rotation %d, resetting to rotation none",
				SensorString(), _device_id, _configuration_index, rotation_value);
			rotation_value = ROTATION_NONE;
			SetConfigurationParam(ParamPrefix::SENS, SensorString(), "ROT", _configuration_index, rotation_value);
		}

		_rotation_enum = static_cast<Rotation>(rotation_value);
		_rotation = get_rot_matrix(_rotation_enum);

		// SENS_FLOWx_PRIO
		_priority = GetConfigurationParam(ParamPrefix::SENS, SensorString(), "PRIO", _configuration_index);

		if ((_priority < 0) || (_priority > 100)) {
			// reset to default, -1 is the uninitialized parameter value
			int32_t new_priority = DEFAULT_PRIORITY;

			if (_priority != -1) {
				PX4_ERR("%s %d (%d) invalid priority %d, resetting to %d", SensorString(), _device_id, _configuration_index, _priority,
					new_priority);
			}

			SetConfigurationParam(ParamPrefix::CAL, SensorString(), "PRIO", _configuration_index, new_priority);
			_priority = new_priority;
		}


		// SENS_FLOWx_POS{X,Y,Z}
		const Vector3f position = GetConfigurationParamsVector3f(ParamPrefix::CAL, SensorString(), "POS",
					  _configuration_index);

		if (Vector3f(_position - position).norm_squared() > 0.001f * 0.001f) {
			_position = position;
		}

	} else {
		Reset();
	}
}

bool OpticalFlowConfiguration::ParametersSave()
{
	if (_configuration_index >= 0) {
		// save configuration
		bool success = true;
		success &= SetConfigurationParam(ParamPrefix::SENS, SensorString(), "ID", _configuration_index, _device_id);
		success &= SetConfigurationParam(ParamPrefix::SENS, SensorString(), "PRIO", _configuration_index, _priority);
		success &= SetConfigurationParam(ParamPrefix::SENS, SensorString(), "ROT", _configuration_index, _rotation_enum);
		success &= SetConfigurationParamsVector3f(ParamPrefix::SENS, SensorString(), "POS", _configuration_index,
				_position);

		return success;
	}

	return false;
}

void OpticalFlowConfiguration::Reset()
{
	_rotation.setIdentity();

	_priority = DEFAULT_PRIORITY;

	_configuration_index = -1;
}

void OpticalFlowConfiguration::PrintStatus()
{
	PX4_INFO("%s %d offset: [%.4f %.4f %.4f]", SensorString(), device_id(), (double)_position(0), (double)_position(1),
		 (double)_position(2));
}

} // namespace calibration
