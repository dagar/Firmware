/****************************************************************************
 *
 * Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file df_mpu9250_wrapper.cpp
 * Lightweight driver to access the MPU9250 of the DriverFramework.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include <px4_config.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <px4_getopt.h>
#include <errno.h>

#include <systemlib/err.h>
#include <perf/perf_counter.h>
#include <systemlib/mavlink_log.h>

#include <drivers/drv_hrt.h>

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

#include <mpu9250/MPU9250.hpp>
#include <DevMgr.hpp>

#define MPU9250_ACCEL_DEFAULT_RATE 1000
#define MPU9250_GYRO_DEFAULT_RATE 1000

#define MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ 30
#define MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ 30

#define MPU9250_PUB_RATE 280


extern "C" { __EXPORT int df_mpu9250_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;


class DfMpu9250Wrapper : public MPU9250
{
public:
	DfMpu9250Wrapper(bool mag_enabled, enum Rotation rotation);
	~DfMpu9250Wrapper();

	/**
	 * Start automatic measurement.
	 *
	 * @return 0 on success
	 */
	int		start();

	/**
	 * Stop automatic measurement.
	 *
	 * @return 0 on success
	 */
	int		stop();

	/**
	 * Print some debug info.
	 */
	void		info();

private:
	int _publish(struct imu_sensor_data &data);

	PX4Accelerometer	_px4_accel;
	PX4Gyroscope		_px4_gyro;
	PX4Magnetometer		_px4_mag;

	perf_counter_t		    _read_counter;
	perf_counter_t		    _error_counter;
	perf_counter_t		    _fifo_overflow_counter;
	perf_counter_t		    _fifo_corruption_counter;
	perf_counter_t		    _gyro_range_hit_counter;
	perf_counter_t		    _accel_range_hit_counter;
	perf_counter_t		    _mag_fifo_overflow_counter;
	perf_counter_t		    _publish_perf;

	bool _mag_enabled;

};

DfMpu9250Wrapper::DfMpu9250Wrapper(bool mag_enabled, enum Rotation rotation) :
	MPU9250(IMU_DEVICE_PATH, mag_enabled),
	_px4_accel(m_id.dev_id, ORB_PRIO_HIGH, rotation),
	_px4_gyro(m_id.dev_id, ORB_PRIO_HIGH, rotation),
	_px4_mag(m_id.dev_id, ORB_PRIO_LOW, rotation),
	_read_counter(perf_alloc(PC_COUNT, "mpu9250_reads")),
	_error_counter(perf_alloc(PC_COUNT, "mpu9250_errors")),
	_fifo_overflow_counter(perf_alloc(PC_COUNT, "mpu9250_fifo_overflows")),
	_fifo_corruption_counter(perf_alloc(PC_COUNT, "mpu9250_fifo_corruptions")),
	_gyro_range_hit_counter(perf_alloc(PC_COUNT, "mpu9250_gyro_range_hits")),
	_accel_range_hit_counter(perf_alloc(PC_COUNT, "mpu9250_accel_range_hits")),
	_mag_fifo_overflow_counter(perf_alloc(PC_COUNT, "mpu9250_mag_fifo_overflows")),
	_publish_perf(perf_alloc(PC_ELAPSED, "mpu9250_publish")),
	_mag_enabled(mag_enabled)
{
	_px4_accel.set_scale(1.0f / 1000.0f);
	_px4_gyro.set_scale(1.0f / 1000.0f);
	_px4_mag.set_scale(1.0f / 1000.0f);
}

DfMpu9250Wrapper::~DfMpu9250Wrapper()
{
	perf_free(_read_counter);
	perf_free(_error_counter);
	perf_free(_fifo_overflow_counter);
	perf_free(_fifo_corruption_counter);
	perf_free(_gyro_range_hit_counter);
	perf_free(_accel_range_hit_counter);

	if (_mag_enabled) {
		perf_free(_mag_fifo_overflow_counter);
	}

	perf_free(_publish_perf);
}

int DfMpu9250Wrapper::start()
{
	/* Init device and start sensor. */
	int ret = init();

	if (ret != 0) {
		PX4_ERR("MPU9250 init fail: %d", ret);
		return ret;
	}

	ret = MPU9250::start();

	if (ret != 0) {
		PX4_ERR("MPU9250 start fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfMpu9250Wrapper::stop()
{
	/* Stop sensor. */
	int ret = MPU9250::stop();

	if (ret != 0) {
		PX4_ERR("MPU9250 stop fail: %d", ret);
		return ret;
	}

	return 0;
}

void DfMpu9250Wrapper::info()
{
	perf_print_counter(_read_counter);
	perf_print_counter(_error_counter);
	perf_print_counter(_fifo_overflow_counter);
	perf_print_counter(_fifo_corruption_counter);
	perf_print_counter(_gyro_range_hit_counter);
	perf_print_counter(_accel_range_hit_counter);

	if (_mag_enabled) {
		perf_print_counter(_mag_fifo_overflow_counter);
	}

	perf_print_counter(_publish_perf);
}

int DfMpu9250Wrapper::_publish(struct imu_sensor_data &data)
{
	perf_begin(_publish_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();

	// Update all the counters.
	perf_set_count(_read_counter, data.read_counter);
	perf_set_count(_error_counter, data.error_counter);
	perf_set_count(_fifo_overflow_counter, data.fifo_overflow_counter);
	perf_set_count(_fifo_corruption_counter, data.fifo_overflow_counter);
	perf_set_count(_gyro_range_hit_counter, data.gyro_range_hit_counter);
	perf_set_count(_accel_range_hit_counter, data.accel_range_hit_counter);

	_px4_accel.update(timestamp_sample, data.accel_m_s2_x, data.accel_m_s2_y, data.accel_m_s2_z);
	_px4_gyro.update(timestamp_sample, data.gyro_rad_s_x, data.gyro_rad_s_y, data.gyro_rad_s_z);

	if (_mag_enabled) {
		_px4_mag.update(timestamp_sample, data.mag_ga_x, data.mag_ga_y, data.mag_ga_z);
	}

	perf_end(_publish_perf);

	// TODO: check the return codes of this function
	return 0;
};


namespace df_mpu9250_wrapper
{

DfMpu9250Wrapper *g_dev = nullptr;

int start(bool mag_enabled, enum Rotation rotation);
int stop();
int info();
void usage();

int start(bool mag_enabled, enum Rotation rotation)
{
	g_dev = new DfMpu9250Wrapper(mag_enabled, rotation);

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfMpu9250Wrapper object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfMpu9250Wrapper start failed");
		return ret;
	}

	// Open the IMU sensor
	DevHandle h;
	DevMgr::getHandle(IMU_DEVICE_PATH, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)",
			    IMU_DEVICE_PATH, h.getError());
		return -1;
	}

	DevMgr::releaseHandle(h);

	return 0;
}

int stop()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	int ret = g_dev->stop();

	if (ret != 0) {
		PX4_ERR("driver could not be stopped");
		return ret;
	}

	delete g_dev;
	g_dev = nullptr;
	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_INFO("state @ %p", g_dev);
	g_dev->info();

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: df_mpu9250_wrapper 'start', 'start_without_mag', 'info', 'stop'");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");
}

} // namespace df_mpu9250_wrapper


int
df_mpu9250_wrapper_main(int argc, char *argv[])
{
	int ch;
	enum Rotation rotation = ROTATION_NONE;
	int ret = 0;
	int myoptind = 1;
	const char *myoptarg = NULL;

	/* jump over start/off/etc and look at options first */
	while ((ch = px4_getopt(argc, argv, "R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			df_mpu9250_wrapper::usage();
			return 0;
		}
	}

	if (argc <= 1) {
		df_mpu9250_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start_without_mag")) {
		ret = df_mpu9250_wrapper::start(false, rotation);
	}

	else if (!strcmp(verb, "start")) {
		ret = df_mpu9250_wrapper::start(true, rotation);
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_mpu9250_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_mpu9250_wrapper::info();
	}

	else {
		df_mpu9250_wrapper::usage();
		return 1;
	}

	return ret;
}
