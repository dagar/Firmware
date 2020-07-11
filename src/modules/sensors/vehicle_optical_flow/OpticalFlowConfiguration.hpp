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

#pragma once

#include <lib/conversion/rotation.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <uORB/Subscription.hpp>

namespace sensors
{
class OpticalFlowConfiguration
{
public:
	static constexpr int MAX_SENSOR_COUNT = 4;

	static constexpr uint8_t DEFAULT_PRIORITY = 50;

	static constexpr const char *SensorString() { return "FLOW"; }

	OpticalFlowConfiguration();
	explicit OpticalFlowConfiguration(uint32_t device_id);

	~OpticalFlowConfiguration() = default;

	void PrintStatus();

	void set_configuration_index(uint8_t index) { _configuration_index = index; }
	void set_device_id(uint32_t device_id);

	uint32_t device_id() const { return _device_id; }
	bool enabled() const { return (_priority > 0); }
	const matrix::Vector3f &position() const { return _position; }
	const int32_t &priority() const { return _priority; }
	const matrix::Dcmf &rotation() const { return _rotation; }
	const Rotation &rotation_enum() const { return _rotation_enum; }

	// rotate corrected measurements from sensor to body frame
	inline matrix::Vector3f Correct(const matrix::Vector3f &data) const
	{
		return _rotation * matrix::Vector3f{data};
	}

	bool ParametersSave();
	void ParametersUpdate();

	void Reset();

private:
	Rotation _rotation_enum{ROTATION_NONE};

	matrix::Dcmf _rotation;
	matrix::Vector3f _position;

	int8_t _configuration_index{-1};
	uint32_t _device_id{0};
	int32_t _priority{-1};
};
} // namespace sensors
