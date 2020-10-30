/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "../PreFlightCheck.hpp"

#include <HealthFlags.h>

#include <lib/parameters/param.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensors_status.h>

bool PreFlightCheck::imuConsistencyCheck(orb_advert_t *mavlink_log_pub, vehicle_status_s &status,
		const bool report_status)
{
	float accel_test_limit = 1.f;
	param_get(param_find("COM_ARM_IMU_ACC"), &accel_test_limit);

	float gyro_test_limit = 1.f;
	param_get(param_find("COM_ARM_IMU_GYR"), &gyro_test_limit);

	// Get sensors status data if available and exit with a fail recorded if not
	uORB::SubscriptionData<sensors_status_s> sensors_status_accel_sub{ORB_ID(sensors_status_accel)};
	const sensors_status_s &accel = sensors_status_accel_sub.get();

	// Use the difference between IMU's to detect a bad calibration.
	// If a single IMU is fitted, the value being checked will be zero so this check will always pass.
	for (unsigned i = 0; i < (sizeof(accel.inconsistency) / sizeof(accel.inconsistency[0])); i++) {
		if (accel.device_ids[i] != 0) {
			if (accel.device_ids[i] == accel.device_id_primary) {
				set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_ACC, accel.healthy[i], status);

			} else {
				set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_ACC2, accel.healthy[i], status);
			}

			const float inconsistency = accel.inconsistency[i];

			if (inconsistency > accel_test_limit) {
				if (report_status) {
					mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Accel %u inconsistent - Check Cal", i);

					if (accel.device_ids[i] == accel.device_id_primary) {
						set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_ACC, false, status);

					} else {
						set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_ACC2, false, status);
					}
				}

				return false;

			} else if (inconsistency > accel_test_limit * 0.8f) {
				if (report_status) {
					mavlink_log_info(mavlink_log_pub, "Preflight Advice: Accel %u inconsistent - Check Cal", i);
				}
			}
		}
	}

	// Fail if gyro difference greater than 5 deg/sec and notify if greater than 2.5 deg/sec
	uORB::SubscriptionData<sensors_status_s> sensors_status_gyro_sub{ORB_ID(sensors_status_gyro)};
	const sensors_status_s &gyro = sensors_status_gyro_sub.get();

	for (unsigned i = 0; i < (sizeof(gyro.inconsistency) / sizeof(gyro.inconsistency[0])); i++) {
		if (gyro.device_ids[i] != 0) {
			if (gyro.device_ids[i] == gyro.device_id_primary) {
				set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_GYRO, gyro.healthy[i], status);

			} else {
				set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_GYRO2, gyro.healthy[i], status);
			}

			const float inconsistency = gyro.inconsistency[i];

			if (inconsistency > gyro_test_limit) {
				if (report_status) {
					mavlink_log_critical(mavlink_log_pub, "Preflight Fail: Gyro %u inconsistent - Check Cal", i);

					if (gyro.device_ids[i] == gyro.device_id_primary) {
						set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_GYRO, false, status);

					} else {
						set_health_flags_healthy(subsystem_info_s::SUBSYSTEM_TYPE_GYRO2, false, status);
					}
				}

				return false;

			} else if (inconsistency > gyro_test_limit * 0.5f) {
				if (report_status) {
					mavlink_log_info(mavlink_log_pub, "Preflight Advice: Gyro %u inconsistent - Check Cal", i);
				}
			}
		}
	}

	return true;
}
