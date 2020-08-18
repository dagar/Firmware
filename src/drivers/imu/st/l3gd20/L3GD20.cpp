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

#include "L3GD20.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

L3GD20::L3GD20(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
	       spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio) :
	SPI(MODULE_NAME, nullptr, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
	_drdy_gpio(drdy_gpio),
	_px4_gyro(get_device_id(), ORB_PRIO_HIGH, rotation)
{
	set_device_type(DRV_GYR_DEVTYPE_L3GD20);

	_px4_gyro.set_device_type(DRV_GYR_DEVTYPE_L3GD20);
	_px4_gyro.set_temperature(NAN); // actual temperature reading unavailable (OUT_TEMP uncalibrated)

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

L3GD20::~L3GD20()
{
	perf_free(_transfer_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_interval_perf);
}

int L3GD20::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool L3GD20::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void L3GD20::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void L3GD20::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("FIFO empty interval: %d us (%.3f Hz)", _fifo_empty_interval_us,
		 static_cast<double>(1000000 / _fifo_empty_interval_us));

	perf_print_counter(_transfer_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_interval_perf);

	_px4_gyro.print_status();
}

int L3GD20::probe()
{
	const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami != WHOAMI) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void L3GD20::RunImpl()
{
	switch (_state) {
	case STATE::RESET:
		// CTRL_REG5
		RegisterSetBits(Register::CTRL_REG5, CTRL_REG5_BIT::BOOT);
		_reset_timestamp = hrt_absolute_time();
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(15_ms);
		break;

	case STATE::WAIT_FOR_RESET:

		if (RegisterRead(Register::WHO_AM_I) == WHOAMI) {
			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			ScheduleNow();

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 100_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			_state = STATE::FIFO_READ;

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(10_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			}

			FIFOReset();

		} else {
			PX4_DEBUG("Configure failed, retrying");
			// try again in 10 ms
			ScheduleDelayed(10_ms);
		}

		break;

	case STATE::FIFO_READ: {
			hrt_abstime timestamp_sample = 0;

			if (_data_ready_interrupt_enabled) {
				// re-schedule as watchdog timeout
				ScheduleDelayed(10_ms);
				timestamp_sample = _fifo_watermark_interrupt_timestamp;
			}

			if (!_data_ready_interrupt_enabled || (hrt_elapsed_time(&timestamp_sample) > (_fifo_empty_interval_us / 2))) {
				timestamp_sample = hrt_absolute_time();
			}

			// check for FIFO status and count
			const uint8_t FIFO_SRC_REG = RegisterRead(Register::FIFO_SRC_REG);
			const uint8_t samples = FIFO_SRC_REG & 0b11111; // FSS4-FSS0 FIFO stored data level

			bool failure = false;

			if (FIFO_SRC_REG & FIFO_SRC_REG_BIT::OVRN) {
				// overflow
				perf_count(_fifo_overflow_perf);
				FIFOReset();
				return;
			}

			if (samples > FIFO_MAX_SAMPLES) {
				// not technically an overflow, but more samples than we expected or can publish
				perf_count(_fifo_overflow_perf);
				failure = true;
				FIFOReset();

			} else if (samples >= 1) {
				if (!FIFORead(timestamp_sample, samples)) {
					failure = true;
					_px4_gyro.increase_error_count();
				}

			} else if (samples == 0) {
				failure = true;
				perf_count(_fifo_empty_perf);
			}

			if (failure || hrt_elapsed_time(&_last_config_check_timestamp) > 10_ms) {
				// check registers incrementally
				if (RegisterCheck(_register_cfg[_checked_register], true)) {
					_last_config_check_timestamp = timestamp_sample;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reconfigure
					PX4_DEBUG("Health check failed, reconfiguring");
					_state = STATE::CONFIGURE;
					ScheduleNow();
				}
			}
		}

		break;
	}
}

void L3GD20::ConfigureSampleRate(int sample_rate)
{
	if (sample_rate == 0) {
		sample_rate = ST_L3GD20::ODR; // default to max ODR (760 Hz)
	}

	// round down to nearest FIFO sample dt
	const float min_interval = FIFO_SAMPLE_DT;
	float fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval,
				       min_interval);

	_fifo_gyro_samples = math::constrain(fifo_empty_interval_us / (1e6f / GYRO_RATE), 1.f, (float)FIFO_MAX_SAMPLES);

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);

	_px4_gyro.set_update_rate(1e6f / _fifo_empty_interval_us);

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}

void L3GD20::ConfigureFIFOWatermark(uint8_t samples)
{
	if (samples == 1) {
		// enable data ready interrupt if only 1 sample
		for (auto &r : _register_cfg) {
			if (r.reg == Register::CTRL_REG3) {
				r.set_bits |= CTRL_REG3_BIT::I2_DRDY;
				break;
			}
		}

	} else {
		const uint16_t fifo_watermark_threshold = samples;

		for (auto &r : _register_cfg) {
			if (r.reg == Register::FIFO_CTRL_REG) {
				// WTM4-WTM0
				r.set_bits |= (fifo_watermark_threshold & 0b11111);

			} else if (r.reg == Register::CTRL_REG3) {
				// enable FIFO watermark interrupt
				r.set_bits |= CTRL_REG3_BIT::I2_WTM;
			}
		}
	}
}

bool L3GD20::Configure()
{
	bool success = true;

	for (const auto &reg : _register_cfg) {
		if (!RegisterCheck(reg)) {
			success = false;
		}
	}

	_px4_gyro.set_scale(math::radians(70.f / 1000.f)); // 70 mdps/LSB
	_px4_gyro.set_range(math::radians(2000.f));

	return success;
}

int L3GD20::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<L3GD20 *>(arg)->DataReady();
	return 0;
}

void L3GD20::DataReady()
{
	perf_count(_drdy_interval_perf);
	_fifo_watermark_interrupt_timestamp = hrt_absolute_time();
	_fifo_read_samples.store(_fifo_gyro_samples);
	ScheduleNow();
}

bool L3GD20::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on rising edge
	return px4_arch_gpiosetevent(_drdy_gpio, true, false, true, &L3GD20::DataReadyInterruptCallback, this) == 0;
}

bool L3GD20::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool L3GD20::RegisterCheck(const register_config_t &reg_cfg, bool notify)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	if (!success) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);

		if (notify) {
			perf_count(_bad_register_perf);
			_px4_gyro.increase_error_count();
		}
	}

	return success;
}

uint8_t L3GD20::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void L3GD20::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void L3GD20::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = orig_val;

	if (setbits) {
		val |= setbits;
	}

	if (clearbits) {
		val &= ~clearbits;
	}

	RegisterWrite(reg, val);
}

bool L3GD20::FIFORead(const hrt_abstime &timestamp_sample, uint16_t samples)
{
	perf_begin(_transfer_perf);
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 1, FIFO::SIZE);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_end(_transfer_perf);
		perf_count(_bad_transfer_perf);
		return false;
	}

	perf_end(_transfer_perf);

	ProcessGyro(timestamp_sample, buffer, samples);

	return true;
}

void L3GD20::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// FIFO_CTRL_REG: switch to bypass mode to restart data collection
	RegisterClearBits(Register::FIFO_CTRL_REG, FIFO_CTRL_REG_BIT::Bypass_mode);

	// reset while FIFO is disabled
	_fifo_watermark_interrupt_timestamp = 0;
	_fifo_read_samples.store(0);

	// FIFO_CTRL_REG: mode + watermark
	RegisterSetBits(Register::FIFO_CTRL_REG, FIFO_CTRL_REG_BIT::FIFO_mode);
}

void L3GD20::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFOTransferBuffer &buffer,
			 const uint8_t samples)
{
	PX4Gyroscope::FIFOSample gyro;
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = _fifo_empty_interval_us / _fifo_gyro_samples;

	for (int i = 0; i < samples; i++) {
		const FIFO::DATA &fifo_sample = buffer.f[i];

		const int16_t gyro_x = combine(fifo_sample.OUT_X_H, fifo_sample.OUT_X_L);
		const int16_t gyro_y = combine(fifo_sample.OUT_Y_H, fifo_sample.OUT_Y_L);
		const int16_t gyro_z = combine(fifo_sample.OUT_Z_H, fifo_sample.OUT_Z_L);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = gyro_x;
		gyro.y[i] = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
		gyro.z[i] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;
	}

	_px4_gyro.updateFIFO(gyro);
}
