/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file ms5611_spi.cpp
 *
 * SPI interface for MS5611
 */


#include "MS5611_SPI.hpp"

MS5611_SPI::MS5611_SPI(uint8_t bus, uint32_t device) :
	SPI("MS5611_SPI", nullptr, bus, device, SPIDEV_MODE3, 20 * 1000 * 1000)
{
}

int MS5611_SPI::init()
{
	int ret = SPI::init();

	if (ret != OK) {
		PX4_DEBUG("SPI init failed");
		goto out;
	}

	/* send reset command */
	ret = _reset();

	if (ret != OK) {
		PX4_DEBUG("reset failed");
		goto out;
	}

	/* read PROM */
	ret = _read_prom();

	if (ret != OK) {
		PX4_DEBUG("prom readout failed");
		goto out;
	}

out:
	return ret;
}

int MS5611_SPI::read(unsigned offset, void *data, unsigned count)
{
	union _cvt {
		uint8_t	b[4];
		uint32_t w;
	} *cvt = (_cvt *)data;
	uint8_t buf[4] = { 0 | DIR_WRITE, 0, 0, 0 };

	/* read the most recent measurement */
	int ret = _transfer(&buf[0], &buf[0], sizeof(buf));

	if (ret == OK) {
		/* fetch the raw value */
		cvt->b[0] = buf[3];
		cvt->b[1] = buf[2];
		cvt->b[2] = buf[1];
		cvt->b[3] = 0;

		ret = count;
	}

	return ret;
}

int MS5611_SPI::_reset()
{
	uint8_t cmd = ADDR_RESET_CMD | DIR_WRITE;
	return  _transfer(&cmd, nullptr, 1);
}

int MS5611_SPI::_measure(unsigned addr)
{
	uint8_t cmd = addr | DIR_WRITE;
	return _transfer(&cmd, nullptr, 1);
}

uint16_t MS5611_SPI::_reg16(unsigned reg)
{
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };
	transfer(cmd, cmd, sizeof(cmd));
	return (uint16_t)(cmd[1] << 8) | cmd[2];
}

#endif /* PX4_SPIDEV_BARO || PX4_SPIDEV_EXT_BARO */
