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

namespace uORB
{

// Base subscription wrapper class
class Subscription
{
public:

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param instance The instance for multi sub.
	 */
	Subscription() = default;
	Subscription(const orb_metadata *meta, uint8_t instance = 0) : _meta(meta), _instance(instance)
	{
		init();
	}

	virtual ~Subscription() { unsubscribe(); }

	bool init();
	bool forceInit();

	bool valid() const { return _node != nullptr; }
	bool published() { return valid() ? _node->is_published() : init(); }

	/**
	 * Check if there is a new update.
	 * */
	virtual bool updated() { return published() ? (_node->published_message_count() != _last_generation) : false; }

	/**
	 * Update the struct
	 * @param data The uORB message struct we are updating.
	 */
	virtual bool update(void *dst) { return updated() ? copy(dst) : false; }

	/**
	 * Check if subscription updated based on timestamp.
	 *
	 * @return true only if topic was updated based on a timestamp and
	 * copied to buffer successfully.
	 * If topic was not updated since last check it will return false but
	 * still copy the data.
	 * If no data available data buffer will be filled with zeros.
	 */
	bool update(uint64_t *time, void *dst);

	/**
	 * Copy the struct
	 * @param data The uORB message struct we are updating.
	 */
	bool copy(void *dst) { return published() ? _node->copy(dst, _last_generation) : false; }

	hrt_abstime	last_update() { return published() ? _node->last_update() : 0; }

	uint8_t		get_instance() const { return _instance; }
	orb_id_t	get_topic() const { return _meta; }

protected:

	bool subscribe();
	void unsubscribe();

	DeviceNode		*_node{nullptr};
	const orb_metadata	*_meta{nullptr};

	/**
	 * Subscription's latest data generation.
	 * Also used to track (and rate limit) subscription
	 * attempts if the topic has not yet been published.
	 */
	unsigned		_last_generation{0};
	uint8_t			_instance{0};
};

// Subscription wrapper class with configured interval
class SubscriptionInterval : public Subscription
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param interval  The minimum interval in milliseconds between updates
	 * @param instance The instance for multi sub.
	 */
	SubscriptionInterval() = default;
	SubscriptionInterval(const orb_metadata *meta, unsigned interval = 0, uint8_t instance = 0) :
		Subscription(meta, instance),
		_interval(interval)
	{}

	virtual ~SubscriptionInterval() = default;

	bool updated() override
	{
		if (hrt_elapsed_time(&_last_update) >= (_interval * 1000)) {
			return Subscription::updated();
		}

		return false;
	}

	bool update(void *dst) override
	{
		if (updated()) {
			if (copy(dst)) {
				_last_update = hrt_absolute_time();
				return true;
			}
		}

		return false;
	}

	int get_interval() const { return _interval; }
	void set_interval(unsigned interval) { _interval = interval; }

protected:
	uint64_t _last_update{0};	// last update in microseconds
	unsigned _interval{0};		// interval in milliseconds
};

// Subscription wrapper class with data
template<class T>
class SubscriptionData : public Subscription
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param instance The instance for multi sub.
	 */
	SubscriptionData(const orb_metadata *meta, uint8_t instance = 0) :
		Subscription(meta, instance)
	{
		copy(&_data);
	}

	virtual ~SubscriptionData() = default;

	// update the embedded struct.
	bool update() { return Subscription::update((void *)(&_data)); }

	const T &get() const { return _data; }

private:

	T _data{};
};

// Subscription wrapper class with data and configured interval
template<class T>
class SubscriptionIntervalData : public SubscriptionInterval
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID() macro) for the topic.
	 * @param interval  The minimum interval in milliseconds between updates
	 * @param instance The instance for multi sub.
	 */
	SubscriptionIntervalData(const orb_metadata *meta, unsigned interval = 0, uint8_t instance = 0) :
		SubscriptionInterval(meta, interval, instance)
	{
		copy(&_data);
	}

	~SubscriptionIntervalData() override = default;

	// update the embedded struct.
	bool update() { return SubscriptionInterval::update((void *)(&_data)); }

	const T &get() const { return _data; }

private:
	T _data{};
};

} // namespace uORB
