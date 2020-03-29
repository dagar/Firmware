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

/**
 * @file LSM303D.hpp
 *
 * Driver for the ST LSM303D connected via SPI.
 *
 */

#pragma once

#include "ST_LSM303D_Registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/ecl/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace ST_LSM303D;

class LSM303D : public device::SPI, public I2CSPIDriver<LSM303D>
{
public:
	LSM303D(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio);
	~LSM303D() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

	void Start();
	bool Reset();

protected:
	void custom_method(const BusCLIArguments &cli) override;
	void exit_and_cleanup() override;
private:
	// Sensor Configuration
	static constexpr float FIFO_SAMPLE_DT{1e6f / ST_LSM303D::LA_ODR};
	static constexpr float ACCEL_RATE{ST_LSM303D::LA_ODR};

	static constexpr uint32_t FIFO_MAX_SAMPLES{math::min(FIFO::SIZE / sizeof(FIFO::DATA), sizeof(PX4Accelerometer::FIFOSample::x) / sizeof(PX4Accelerometer::FIFOSample::x[0]))};

	// Transfer data
	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::OUT_X_L_A) | DIR_READ | AUTO_INCREMENT};
		FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	};
	// ensure no struct padding
	static_assert(sizeof(FIFOTransferBuffer) == (1 + FIFO_MAX_SAMPLES *sizeof(FIFO::DATA)));

	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Configure();
	void ConfigureSampleRate(int sample_rate);
	void ConfigureFIFOWatermark(uint8_t samples);

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	bool RegisterCheck(const register_config_t &reg_cfg, bool notify = false);

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);
	void RegisterSetBits(Register reg, uint8_t setbits) { RegisterSetAndClearBits(reg, setbits, 0); }
	void RegisterClearBits(Register reg, uint8_t clearbits) { RegisterSetAndClearBits(reg, 0, clearbits); }

	bool FIFORead(const hrt_abstime &timestamp_sample, uint16_t samples);
	void FIFOReset();

	void ProcessAccel(const hrt_abstime &timestamp_sample, const FIFOTransferBuffer &buffer, const uint8_t samples);

	const spi_drdy_gpio_t _drdy_gpio;

	PX4Accelerometer _px4_accel;
	PX4Magnetometer _px4_mag;

	perf_counter_t _transfer_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": transfer")};
	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};
	perf_counter_t _drdy_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": DRDY interval")};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	hrt_abstime _fifo_watermark_interrupt_timestamp{0};

	px4::atomic<uint8_t> _fifo_read_samples{0};
	bool _data_ready_interrupt_enabled{false};
	bool _force_fifo_count_check{true};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		FIFO_READ,
	};

	STATE _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1000}; // default 1000 us / 1000 Hz transfer interval
	uint8_t _fifo_accel_samples{static_cast<uint8_t>(_fifo_empty_interval_us / (1000000 / ACCEL_RATE))};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{8};
	register_config_t _register_cfg[size_register_cfg] {
		// Register                | Set bits, Clear bits
		{ Register::CTRL0,         CTRL0_BIT::FIFO_EN | CTRL0_BIT::FTH_EN, CTRL0_BIT::BOOT },
		{ Register::CTRL1,         CTRL1_BIT::AODR_1600Hz | CTRL1_BIT::BDU, 0 },
		{ Register::CTRL2,         CTRL2_BIT::AFS_16G, 0 },
		{ Register::CTRL4,         0, 0 },
		{ Register::CTRL5,         CTRL5_BIT::TEMP_EN | CTRL5_BIT::M_RES_HIGH | CTRL5_BIT::M_ODR_50_HZ, Bit3 },
		{ Register::CTRL6,         CTRL6_BIT::MFS_12_GAUSS, 0 },
		{ Register::CTRL7,         0, CTRL7_BIT::MD },
		{ Register::FIFO_CTRL,     FIFO_CTRL_BIT::FIFO_mode, 0 },
	};
};
