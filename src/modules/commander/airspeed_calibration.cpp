/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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

/**
 * @file airspeed_calibration.cpp
 * Airspeed sensor calibration routine
 */

#include "airspeed_calibration.h"
#include "calibration_messages.h"
#include "calibration_routines.h"
#include "commander_helper.h"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/AlphaFilter/AlphaFilter.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <lib/parameters/param.h>
#include <uORB/SubscriptionBlocking.hpp>
#include <uORB/topics/differential_pressure.h>

static constexpr char sensor_name[] {"airspeed"};

static void feedback_calibration_failed(orb_advert_t *mavlink_log_pub)
{
	px4_sleep(5);
	calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, sensor_name);
}

int do_airspeed_calibration(orb_advert_t *mavlink_log_pub)
{
	const hrt_abstime calibration_started = hrt_absolute_time();

	unsigned calibration_counter = 0;
	const unsigned maxcount = 2400;
	const unsigned calibration_count = (maxcount * 2) / 3;

	/* give directions */
	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, sensor_name);

	uORB::SubscriptionBlocking<differential_pressure_s> diff_pres_sub{ORB_ID(differential_pressure)};

	float diff_pres_offset = 0.0f;

	calibration_log_critical(mavlink_log_pub, "[cal] Ensure sensor is not measuring wind");
	px4_usleep(500 * 1000);

	while (calibration_counter < calibration_count) {

		if (calibrate_cancel_check(mavlink_log_pub, calibration_started)) {
			return PX4_ERROR;
		}

		differential_pressure_s diff_pres;

		if (diff_pres_sub.updateBlocking(diff_pres, 100000)) {

			diff_pres_offset += diff_pres.differential_pressure_raw_pa;
			calibration_counter++;

			/* any differential pressure failure a reason to abort */
			if (diff_pres.error_count != 0) {
				calibration_log_critical(mavlink_log_pub, "[cal] Airspeed sensor is reporting errors (%d)", diff_pres.error_count);
				calibration_log_critical(mavlink_log_pub, "[cal] Check wiring, reboot vehicle, and try again");
				feedback_calibration_failed(mavlink_log_pub);
				return PX4_ERROR;
			}

			if (calibration_counter % (calibration_count / 20) == 0) {
				calibration_log_info(mavlink_log_pub, CAL_QGC_PROGRESS_MSG, (calibration_counter * 80) / calibration_count);
			}

		} else {
			/* any poll failure for 1s is a reason to abort */
			feedback_calibration_failed(mavlink_log_pub);
			return PX4_ERROR;
		}
	}

	diff_pres_offset = diff_pres_offset / calibration_count;

	if (PX4_ISFINITE(diff_pres_offset)) {

		// Prevent a completely zero param
		// since this is used to detect a missing calibration
		// This value is numerically down in the noise and has
		// no effect on the sensor performance.
		if (fabsf(diff_pres_offset) < 0.00000001f) {
			diff_pres_offset = 0.00000001f;
		}

		if (param_set(param_find("SENS_DPRES_OFF"), &diff_pres_offset)) {
			calibration_log_critical(mavlink_log_pub, CAL_ERROR_SET_PARAMS_MSG);
			return PX4_ERROR;
		}

	} else {
		feedback_calibration_failed(mavlink_log_pub);
		return PX4_ERROR;
	}

	calibration_log_info(mavlink_log_pub, "[cal] Offset of %.3f Pascal", (double)diff_pres_offset);

	/* wait 100 ms to ensure parameter propagated through the system */
	px4_usleep(100 * 1000);

	calibration_log_critical(mavlink_log_pub, "[cal] Blow across front of pitot without touching");

	AlphaFilter<float> diff_pres_filtered{0.1f};

	/* just take a few samples and make sure pitot tubes are not reversed, timeout after ~30 seconds */
	while (calibration_counter < maxcount) {

		if (calibrate_cancel_check(mavlink_log_pub, calibration_started)) {
			return PX4_ERROR;
		}

		differential_pressure_s diff_pres;

		if (diff_pres_sub.updateBlocking(diff_pres, 100000)) {

			diff_pres_filtered.update(diff_pres.differential_pressure_raw_pa - diff_pres_offset);

			if (fabsf(diff_pres_filtered.getState()) > 50.0f) {
				if (diff_pres_filtered.getState() > 0) {
					calibration_log_info(mavlink_log_pub, "[cal] Positive pressure: OK (%.3f Pa)", (double)diff_pres_filtered.getState());
					break;

				} else {
					/* do not allow negative values */
					calibration_log_critical(mavlink_log_pub, "[cal] Negative pressure difference detected (%.3f Pa)",
								 (double)diff_pres_filtered.getState());
					calibration_log_critical(mavlink_log_pub, "[cal] Swap static and dynamic ports!");

					/* the user setup is wrong, wipe the calibration to force a proper re-calibration */
					diff_pres_offset = 0.0f;

					if (param_set(param_find("SENS_DPRES_OFF"), &diff_pres_offset)) {
						calibration_log_critical(mavlink_log_pub, CAL_ERROR_SET_PARAMS_MSG);
						return PX4_ERROR;
					}

					/* save */
					calibration_log_info(mavlink_log_pub, CAL_QGC_PROGRESS_MSG, 0);

					feedback_calibration_failed(mavlink_log_pub);
					return PX4_ERROR;
				}
			}

			if (calibration_counter % 500 == 0) {
				calibration_log_info(mavlink_log_pub, "[cal] Create air pressure! (got %.3f, wanted: 50 Pa)",
						     (double)diff_pres_filtered.getState());
				tune_neutral(true);
			}

			calibration_counter++;

		} else {
			/* any poll failure for 1s is a reason to abort */
			feedback_calibration_failed(mavlink_log_pub);
			return PX4_ERROR;
		}
	}

	if (calibration_counter == maxcount) {
		feedback_calibration_failed(mavlink_log_pub);
		return PX4_ERROR;
	}

	calibration_log_info(mavlink_log_pub, CAL_QGC_PROGRESS_MSG, 100);

	calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, sensor_name);
	tune_neutral(true);

	return PX4_OK;
}
