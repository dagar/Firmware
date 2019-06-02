/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file Subscription.hpp
 *
 */

#pragma once

#include <uORB/uORB.h>
#include <px4_defines.h>

#include "uORBDeviceNode.hpp"
#include "uORBManager.hpp"
#include "uORBUtils.hpp"

#include "Subscription.hpp"

namespace uORB
{

// Base subscription wrapper class
class SubscriptionInterval
{
public:

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param interval The requested maximum update interval in milliseconds.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionInterval(const orb_metadata *meta, unsigned interval = 0, uint8_t instance = 0) :
		_subscription{meta, instance},
		_interval(interval)
	{
	}
	SubscriptionInterval() : _subscription{nullptr} {}

	~SubscriptionInterval() = default;

	bool ForceInit() { return _subscription.ForceInit(); }

	/**
	 * Check if there is a new update.
	 * */
	bool updated()
	{
		if (hrt_elapsed_time(&_last_update) >= (_interval * 1000)) {
			return _subscription.published();
		}

		return false;
	}

	/**
	 * Update the struct
	 * @param data The uORB message struct we are updating.
	 */
	bool Update(void *dst)
	{
		if (updated()) {
			if (_subscription.Copy(dst)) {
				_last_update = hrt_absolute_time();
				return true;
			}
		}

		return false;
	}

	// Simple accessors
	bool		valid() const { return _subscription.valid(); }
	uint8_t		instance() const { return _subscription.instance(); }
	orb_id_t	topic() const { return _subscription.topic(); }
	uint16_t	interval() const { return _interval; }

	void		set_interval(uint16_t interval) { _interval = interval; }

private:

	Subscription	_subscription;
	uint64_t	_last_update{0};	// last update in microseconds
	uint16_t	_interval{0};		// maximum update interval in milliseconds

};

} // namespace uORB
