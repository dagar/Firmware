/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include <parameters/param.h>

#include <uavcan/uavcan.hpp>
#include <uavcan/protocol/param_server.hpp>

class UavcanNodeParamManager : public uavcan::IParamManager
{
public:
	UavcanNodeParamManager();

	void getParamNameByIndex(Index index, Name &out_name) const override;
	void assignParamValue(const Name &name, const Value &value) override;
	void readParamValue(const Name &name, Value &out_value) const override;
	void readParamDefaultMaxMin(const Name &name, Value &out_default,
				    NumericValue &out_max, NumericValue &out_min) const override;
	int saveAllParams() override;
	int eraseAllParams() override;
private:

	/**
	 * Get parameter index in parameter map.
	 */
	int get_param_index(const char *name) const;

	/**
	 * Get the param_t handle for the mapping at index.
	 */
	param_t get_param_handle(int index) const;

	/**
	 * Initialize the parameter map.
	 */
	int init_parameters();

	/**
	 * Parameter map from UAVCAN indices to param handles.
	 * Must be in alphabetical order by parameter name.
	 */
	struct ParameterMap {
		param_t cannode_bitrate;
		param_t cannode_esc0;
		param_t cannode_esc1;
		param_t cannode_esc10;
		param_t cannode_esc11;
		param_t cannode_esc12;
		param_t cannode_esc13;
		param_t cannode_esc14;
		param_t cannode_esc2;
		param_t cannode_esc3;
		param_t cannode_esc4;
		param_t cannode_esc5;
		param_t cannode_esc6;
		param_t cannode_esc7;
		param_t cannode_esc8;
		param_t cannode_esc9;
		param_t cannode_esc_en;
		param_t cannode_esc_mask;
		param_t cannode_node_id;
		param_t uavcan_baro_t;
		param_t uavcan_mag_t;
		unsigned int param_count;
	} _parameter_map{0};
};
