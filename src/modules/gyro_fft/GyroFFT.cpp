/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "GyroFFT.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

GyroFFT::GyroFFT() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	for (int i = 0; i < MAX_NUM_PEAKS; i++) {
		_sensor_gyro_fft.peak_frequencies_x[i] = NAN;
		_sensor_gyro_fft.peak_frequencies_y[i] = NAN;
		_sensor_gyro_fft.peak_frequencies_z[i] = NAN;

		_sensor_gyro_fft.peak_frequencies_x_raw[i] = NAN;
		_sensor_gyro_fft.peak_frequencies_y_raw[i] = NAN;
		_sensor_gyro_fft.peak_frequencies_z_raw[i] = NAN;

		_sensor_gyro_fft.peak_magnitude_x[i] = NAN;
		_sensor_gyro_fft.peak_magnitude_y[i] = NAN;
		_sensor_gyro_fft.peak_magnitude_z[i] = NAN;
	}
}

GyroFFT::~GyroFFT()
{
	perf_free(_cycle_perf);
	perf_free(_cycle_interval_perf);
	perf_free(_fft_perf);
	perf_free(_gyro_generation_gap_perf);
}

bool GyroFFT::init()
{
	_imu_gyro_fft_len = 64;

	if (!SensorSelectionUpdate(true)) {
		ScheduleDelayed(500_ms);
	}

	return true;
}

bool GyroFFT::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if ((sensor_selection.gyro_device_id != 0) && (_selected_sensor_device_id != sensor_selection.gyro_device_id)) {
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

				if (sensor_gyro_sub.get().device_id == sensor_selection.gyro_device_id) {
					if (_sensor_gyro_sub.ChangeInstance(i) && _sensor_gyro_sub.registerCallback()) {
						//_sensor_gyro_sub.set_required_updates(sensor_gyro_s::ORB_QUEUE_LENGTH - 1);
						_selected_sensor_device_id = sensor_selection.gyro_device_id;
						return true;
					}
				}
			}

			PX4_ERR("unable to find or subscribe to selected sensor (%d)", sensor_selection.gyro_device_id);
		}
	}

	return false;
}

void GyroFFT::VehicleIMUStatusUpdate(bool force)
{
	if (_vehicle_imu_status_sub.updated() || force) {
		vehicle_imu_status_s vehicle_imu_status;

		if (_vehicle_imu_status_sub.copy(&vehicle_imu_status)) {
			// find corresponding vehicle_imu_status instance if the device_id doesn't match
			if (vehicle_imu_status.gyro_device_id != _selected_sensor_device_id) {

				for (uint8_t imu_status = 0; imu_status < MAX_SENSOR_COUNT; imu_status++) {
					uORB::Subscription imu_status_sub{ORB_ID(vehicle_imu_status), imu_status};

					if (imu_status_sub.copy(&vehicle_imu_status)) {
						if (vehicle_imu_status.gyro_device_id == _selected_sensor_device_id) {
							_vehicle_imu_status_sub.ChangeInstance(imu_status);
							break;
						}
					}
				}
			}

			// update gyro sample rate
			if ((vehicle_imu_status.gyro_device_id == _selected_sensor_device_id) && (vehicle_imu_status.gyro_rate_hz > 0)) {
				_gyro_sample_rate_hz = vehicle_imu_status.gyro_rate_hz;
				return;
			}
		}
	}
}

// helper function used for frequency estimation
static float tau(float x)
{
	// tau(x) = 1/4 * log(3x^2 + 6x + 1) – sqrt(6)/24 * log((x + 1 – sqrt(2/3))  /  (x + 1 + sqrt(2/3)))
	float p1 = logf(3.f * powf(x, 2.f) + 6.f * x + 1.f);
	float part1 = x + 1.f - sqrtf(2.f / 3.f);
	float part2 = x + 1.f + sqrtf(2.f / 3.f);
	float p2 = logf(part1 / part2);
	return (0.25f * p1 - sqrtf(6.f) / 24.f * p2);
}

float GyroFFT::EstimatePeakFrequencyBin(int axis, int32_t k)
{
	if (k > 1) {
		// find peak location using Quinn's Second Estimator (2020-06-14: http://dspguru.com/dsp/howtos/how-to-interpolate-fft-peak/)
		const auto &dft = _sliding_dft[axis];

		const float divider = (dft.dft(k).real() * dft.dft(k).real() + dft.dft(k).imag() * dft.dft(k).imag());

		// ap = (X[k + 1].r * X[k].r + X[k+1].i * X[k].i) / (X[k].r * X[k].r + X[k].i * X[k].i)
		float ap = (dft.dft(k + 1).real() * dft.dft(k).real() + dft.dft(k + 1).imag() * dft.dft(k).imag()) / divider;

		// dp = -ap / (1 – ap)
		float dp = -ap  / (1.f - ap);

		// am = (X[k - 1].r * X[k].r + X[k – 1].i * X[k].i) / (X[k].r * X[k].r + X[k].i * X[k].i)
		float am = (dft.dft(k + 1).real() * dft.dft(k).real() + dft.dft(k + 1).imag() * dft.dft(k).imag()) / divider;

		// dm = am / (1 – am)
		float dm = am / (1.f - am);

		// d = (dp + dm) / 2 + tau(dp * dp) – tau(dm * dm)
		float d = (dp + dm) / 2.f + tau(dp * dp) - tau(dm * dm);

		// k’ = k + d
		float adjusted_bin = k + d;

		return adjusted_bin;
	}

	return NAN;
}

void GyroFFT::Run()
{
	if (should_exit()) {
		_sensor_gyro_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// backup schedule
	ScheduleDelayed(500_ms);

	perf_begin(_cycle_perf);
	perf_count(_cycle_interval_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
	}

	const bool selection_updated = SensorSelectionUpdate();
	VehicleIMUStatusUpdate(selection_updated);

	// run on sensor gyro updates
	sensor_gyro_s sensor_gyro;

	while (_sensor_gyro_sub.update(&sensor_gyro)) {
		if (_sensor_gyro_sub.get_last_generation() != _gyro_last_generation + 1) {
			// force reset if we've missed a sample
			perf_count(_gyro_generation_gap_perf);
		}

		_gyro_last_generation = _sensor_gyro_sub.get_last_generation();

		perf_begin(_fft_perf);
		_sliding_dft[0].update(sensor_gyro.x);
		_sliding_dft[1].update(sensor_gyro.y);
		_sliding_dft[2].update(sensor_gyro.z);
		perf_end(_fft_perf);
	}

	Update(sensor_gyro.timestamp_sample);

	perf_end(_cycle_perf);
}

void GyroFFT::Update(const hrt_abstime &timestamp_sample)
{
	bool publish = false;
	const float resolution_hz = _gyro_sample_rate_hz / _imu_gyro_fft_len;

	const float max_freq_hz = math::min(_param_imu_gyro_fft_max.get(),
					    math::min(_gyro_sample_rate_hz / 2.f, _param_imu_gyro_ratemax.get() / 2.f));

	float *peak_frequencies_raw[] {_sensor_gyro_fft.peak_frequencies_x_raw, _sensor_gyro_fft.peak_frequencies_y_raw, _sensor_gyro_fft.peak_frequencies_z_raw};

	float *peak_frequencies_out[] {_sensor_gyro_fft.peak_frequencies_x, _sensor_gyro_fft.peak_frequencies_y, _sensor_gyro_fft.peak_frequencies_z};
	float *peak_magnitude_out[] {_sensor_gyro_fft.peak_magnitude_x, _sensor_gyro_fft.peak_magnitude_y, _sensor_gyro_fft.peak_magnitude_z};
	float *peak_snr_out[] {_sensor_gyro_fft.peak_snr_x, _sensor_gyro_fft.peak_snr_y, _sensor_gyro_fft.peak_snr_z};

	for (int axis = 0; axis < 3; axis++) {
		// if we have enough samples begin processing
		if (_sliding_dft[axis].data_valid()) {

			// sum all
			float bin_mag_sum = 0;

			for (int bucket_index = 1; bucket_index < _imu_gyro_fft_len; bucket_index++) {
				const float real = _sliding_dft[axis].dft(bucket_index).real();
				const float imag = _sliding_dft[axis].dft(bucket_index).imag();

				const float fft_magnitude_squared = real * real + imag * imag;

				bin_mag_sum += fft_magnitude_squared;
			}

			_sensor_gyro_fft.total_energy[axis] = bin_mag_sum;

			// find raw peaks
			int raw_peak_index[MAX_NUM_PEAKS] {};
			float raw_peak_magnitude[MAX_NUM_PEAKS] {};

			for (int bucket_index = 1; bucket_index < _imu_gyro_fft_len; bucket_index++) {
				const float freq_hz = bucket_index * resolution_hz;

				if (freq_hz >= max_freq_hz) {
					break;
				}

				const float real = _sliding_dft[axis].dft(bucket_index).real();
				const float imag = _sliding_dft[axis].dft(bucket_index).imag();

				const float fft_magnitude_squared = real * real + imag * imag;

				float snr = 10.f * log10f((_imu_gyro_fft_len - 1) * fft_magnitude_squared / (bin_mag_sum - fft_magnitude_squared));
				static constexpr float MIN_SNR = 10.f; // TODO: configurable?

				if (snr > MIN_SNR) {
					for (int i = 0; i < MAX_NUM_PEAKS; i++) {
						if (fft_magnitude_squared > raw_peak_magnitude[i]) {
							raw_peak_magnitude[i] = sqrtf(fft_magnitude_squared);
							raw_peak_index[i] = bucket_index;

							peak_frequencies_raw[axis][i] = freq_hz;
							publish = true;

							break;
						}
					}
				}
			}

			int num_peaks_found = 0;
			float peak_frequencies[MAX_NUM_PEAKS] {};
			float peak_magnitude[MAX_NUM_PEAKS] {};
			float peak_snr[MAX_NUM_PEAKS] {};

			// estimate adjusted frequency bin, magnitude, and SNR for the largest peaks found
			for (int i = 0; i < MAX_NUM_PEAKS; i++) {
				if (raw_peak_index[i] > 0) {
					const float adjusted_bin = EstimatePeakFrequencyBin(axis, raw_peak_index[i]);

					if (PX4_ISFINITE(adjusted_bin) && (fabsf(adjusted_bin - raw_peak_index[i]) < 3.f)) {

						const float freq_adjusted = (_gyro_sample_rate_hz * adjusted_bin / _imu_gyro_fft_len);

						if (PX4_ISFINITE(freq_adjusted)
						    && (freq_adjusted >= _param_imu_gyro_fft_min.get())
						    && (freq_adjusted < max_freq_hz)) {

							peak_frequencies[num_peaks_found] = freq_adjusted;
							peak_magnitude[num_peaks_found] = raw_peak_magnitude[i];
							//peak_snr[num_peaks_found] = snr;

							num_peaks_found++;
						}
					}
				}
			}

			if (num_peaks_found > 0) {
				float peak_frequencies_diff[MAX_NUM_PEAKS][MAX_NUM_PEAKS];

				for (int peak_new = 0; peak_new < MAX_NUM_PEAKS; peak_new++) {
					// compute distance to previous peaks
					for (int peak_prev = 0; peak_prev < MAX_NUM_PEAKS; peak_prev++) {
						if (PX4_ISFINITE(peak_frequencies_out[axis][peak_prev]) && (peak_frequencies_out[axis][peak_prev] > 0)) {
							peak_frequencies_diff[peak_new][peak_prev] = fabsf(peak_frequencies[peak_new] - peak_frequencies_out[axis][peak_prev]);

						} else {
							peak_frequencies_diff[peak_new][peak_prev] = INFINITY;
						}
					}
				}

				// go through peak_frequencies_diff and find absolute smallest diff
				//  - copy new peak to old peak slot
				//  - exclude new peak (row) and old peak (column) in search
				//  - repeat
				//
				//  - finally copy unmatched peaks to empty slots
				bool peak_new_copied[MAX_NUM_PEAKS] {};
				bool peak_out_filled[MAX_NUM_PEAKS] {};

				for (int new_peak = 0; new_peak < MAX_NUM_PEAKS; new_peak++) {
					// find slot of published peak that is closest to this new peak
					float smallest_diff = INFINITY;
					int smallest_r = -1; // rows are new peaks
					int smallest_c = -1; // columns are old peaks (published)

					for (int r = 0; r < MAX_NUM_PEAKS; r++) {
						for (int c = 0; c < MAX_NUM_PEAKS; c++) {
							if (!peak_new_copied[r] && !peak_out_filled[c]
							    && (peak_frequencies_diff[r][c] < smallest_diff)) {

								smallest_diff = peak_frequencies_diff[r][c];
								smallest_r = r;
								smallest_c = c;
							}
						}
					}

					// new peak: r old peak: c
					if (PX4_ISFINITE(smallest_diff) && (smallest_diff > 0)) {
						// copy new peak
						peak_frequencies_out[axis][smallest_c] = peak_frequencies[smallest_r];
						peak_magnitude_out[axis][smallest_c] = peak_magnitude[smallest_r];
						peak_snr_out[axis][smallest_c] = peak_snr[smallest_r];

						// clear
						peak_frequencies_diff[smallest_r][smallest_c] = NAN;

						peak_new_copied[smallest_r] = true;
						peak_out_filled[smallest_c] = true;

						_last_update[axis][smallest_c] = timestamp_sample;

						publish = true;
					}
				}

				// copy any remaining new (unmatched) peaks to overwrite old slots
				for (int r = 0; r < MAX_NUM_PEAKS; r++) {
					if (PX4_ISFINITE(peak_frequencies[r]) && (peak_frequencies[r] > 0)) {
						int oldest_slot = -1;
						hrt_abstime oldest = timestamp_sample;

						// find oldest slot and replace with new peak frequency
						for (int c = 0; c < MAX_NUM_PEAKS; c++) {
							if (_last_update[axis][c] < oldest) {
								oldest_slot = c;
								oldest = _last_update[axis][c];
							}
						}

						if (oldest_slot >= 0) {
							// copy peak to output slot
							peak_frequencies_out[axis][oldest_slot] = peak_frequencies[r];
							peak_magnitude_out[axis][oldest_slot] = peak_magnitude[r];
							peak_snr_out[axis][oldest_slot] = peak_snr[r];
							_last_update[axis][oldest_slot] = timestamp_sample;

							publish = true;
						}
					}
				}
			}

			// mark remaining or stale slots empty
			for (int i = 0; i < MAX_NUM_PEAKS; i++) {
				if (!PX4_ISFINITE(peak_frequencies_out[axis][i])
				    || (peak_frequencies_out[axis][i] < _param_imu_gyro_fft_min.get())
				    || (timestamp_sample - _last_update[axis][i] > 1_s)) {

					peak_frequencies_out[axis][i] = NAN;
					peak_magnitude_out[axis][i] = NAN;
					peak_snr_out[axis][i] = NAN;
					_last_update[axis][i] = 0;
				}
			}
		}
	}

	if (publish) {
		_sensor_gyro_fft.timestamp_sample = timestamp_sample;
		_sensor_gyro_fft.device_id = _selected_sensor_device_id;
		_sensor_gyro_fft.sensor_sample_rate_hz = _gyro_sample_rate_hz;
		_sensor_gyro_fft.resolution_hz = resolution_hz;
		_sensor_gyro_fft.timestamp = hrt_absolute_time();
		_sensor_gyro_fft_pub.publish(_sensor_gyro_fft);
	}
}

int GyroFFT::task_spawn(int argc, char *argv[])
{
	GyroFFT *instance = new GyroFFT();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int GyroFFT::print_status()
{
	PX4_INFO("gyro sample rate: %.3f Hz", (double)_gyro_sample_rate_hz);
	perf_print_counter(_cycle_perf);
	perf_print_counter(_cycle_interval_perf);
	perf_print_counter(_fft_perf);
	perf_print_counter(_gyro_generation_gap_perf);
	return 0;
}

int GyroFFT::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GyroFFT::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gyro_fft", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int gyro_fft_main(int argc, char *argv[])
{
	return GyroFFT::main(argc, argv);
}
