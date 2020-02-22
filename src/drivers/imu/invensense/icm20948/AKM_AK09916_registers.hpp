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

/**
 * @file AKM_AK09916_registers.hpp
 *
 * Asahi Kasei Microdevices (AKM) AK09916 registers.
 *
 */

#pragma once

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace AKM_AK09916
{

static constexpr uint8_t WHOAMI = 0b00001001;

enum class Register : uint8_t {
	WIA2  = 0x01,   // Device ID

	ST1   = 0x10,   // Status 1
	HXL   = 0x11,
	HXH   = 0x12,
	HYL   = 0x13,
	HYH   = 0x14,
	HZL   = 0x15,
	HZH   = 0x16,

	ST2   = 0x18,   // Status 2

	CNTL2 = 0x31,   // Control 2
	CNTL3 = 0x32,   // Control 3
};


// ST1
enum ST1_BIT : uint8_t {
	DOR  = Bit1,    // Data overrun
	DRDY = Bit0,    // Data is ready
};

// CNTL2
enum CNTL2_BIT : uint8_t {
	MODE1 = Bit1,        // Continuous measurement mode 1 (10Hz)
	MODE2 = Bit2,        // Continuous measurement mode 2 (20Hz)
	MODE3 = Bit2 | Bit1, // Continuous measurement mode 3 (50Hz)
	MODE4 = Bit3,        // Continuous measurement mode 4 (100Hz)
};

// CNTL3
enum CNTL3_BIT : uint8_t {
	SRST = Bit0,
};

} // namespace AKM_AK09916
