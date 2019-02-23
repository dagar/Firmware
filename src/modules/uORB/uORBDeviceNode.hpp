/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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

#include "uORBCommon.hpp"
#include "uORBDeviceMaster.hpp"

#include <lib/cdev/CDev.hpp>

#include <containers/IntrusiveList.hpp>

namespace uORB
{
class DeviceNode;
class DeviceMaster;
class Manager;
}

/**
 * Per-object device instance.
 */
class uORB::DeviceNode : public cdev::CDev, public IntrusiveListNode<uORB::DeviceNode *>
{
public:
	DeviceNode(const struct orb_metadata *meta, const uint8_t instance, const char *path, uint8_t priority,
		   uint8_t queue_size = 1);
	~DeviceNode();

	// no copy, assignment, move, move assignment
	DeviceNode(const DeviceNode &) = delete;
	DeviceNode &operator=(const DeviceNode &) = delete;
	DeviceNode(DeviceNode &&) = delete;
	DeviceNode &operator=(DeviceNode &&) = delete;

	/**
	 * Method to create a subscriber instance and return the struct
	 * pointing to the subscriber as a file pointer.
	 */
	int open(cdev::file_t *filp) override;

	/**
	 * Method to close a subscriber for this topic.
	 */
	int close(cdev::file_t *filp) override;

	/**
	 * reads data from a subscriber node to the buffer provided.
	 * @param filp
	 *   The subscriber from which the data needs to be read from.
	 * @param buffer
	 *   The buffer into which the data is read into.
	 * @param buflen
	 *   the length of the buffer
	 * @return
	 *   ssize_t the number of bytes read.
	 */
	ssize_t read(cdev::file_t *filp, char *buffer, size_t buflen) override;

	/**
	 * writes the published data to the internal buffer to be read by
	 * subscribers later.
	 * @param filp
	 *   the subscriber; this is not used.
	 * @param buffer
	 *   The buffer for the input data
	 * @param buflen
	 *   the length of the buffer.
	 * @return ssize_t
	 *   The number of bytes that are written
	 */
	ssize_t write(cdev::file_t *filp, const char *buffer, size_t buflen) override;

	/**
	 * IOCTL control for the subscriber.
	 */
	int ioctl(cdev::file_t *filp, int cmd, unsigned long arg) override;

	/**
	 * Method to publish a data to this node.
	 */
	static ssize_t    publish(const orb_metadata *meta, orb_advert_t handle, const void *data);

	static int        unadvertise(orb_advert_t handle);

#ifdef ORB_COMMUNICATOR
	static int16_t topic_advertised(const orb_metadata *meta, int priority);
	//static int16_t topic_unadvertised(const orb_metadata *meta, int priority);

	/**
	 * processes a request for add subscription from remote
	 * @param rateInHz
	 *   Specifies the desired rate for the message.
	 * @return
	 *   0 = success
	 *   otherwise failure.
	 */
	int16_t process_add_subscription(int32_t rateInHz);

	/**
	 * processes a request to remove a subscription from remote.
	 */
	int16_t process_remove_subscription();

	/**
	 * processed the received data message from remote.
	 */
	int16_t process_received_message(int32_t length, uint8_t *data);
#endif /* ORB_COMMUNICATOR */

	/**
	  * Add the subscriber to the node's list of subscriber.  If there is
	  * remote proxy to which this subscription needs to be sent, it will
	  * done via uORBCommunicator::IChannel interface.
	  * @param sd
	  *   the subscriber to be added.
	  */
	void add_internal_subscriber();

	/**
	 * Removes the subscriber from the list.  Also notifies the remote
	 * if there a uORBCommunicator::IChannel instance.
	 * @param sd
	 *   the Subscriber to be removed.
	 */
	void remove_internal_subscriber();

	/**
	 * Return true if this topic has been published.
	 *
	 * This is used in the case of multi_pub/sub to check if it's valid to advertise
	 * and publish to this node or if another node should be tried. */
	bool is_published() const { return _published; }

	/**
	 * Try to change the size of the queue. This can only be done as long as nobody published yet.
	 * This is the case, for example when orb_subscribe was called before an orb_advertise.
	 * The queue size can only be increased.
	 * @param queue_size new size of the queue
	 * @return PX4_OK if queue size successfully set
	 */
	int update_queue_size(unsigned int queue_size);

	/**
	 * Print statistics (nr of lost messages)
	 * @param reset if true, reset statistics afterwards
	 * @return true if printed something, false otherwise (if no lost messages)
	 */
	bool print_statistics(bool reset);

	uint8_t get_queue_size() const { return _queue_size; }

	int8_t subscriber_count() const { return _subscriber_count; }

	uint32_t lost_message_count() const { return _lost_messages; }

	unsigned published_message_count() const { return _generation; }

	const orb_metadata *get_meta() const { return _meta; }

	const char *get_name() const { return _meta->o_name; }

	uint8_t get_instance() const { return _instance; }

	int get_priority() const { return _priority; }
	void set_priority(uint8_t priority) { _priority = priority; }

protected:

	pollevent_t poll_state(cdev::file_t *filp) override;

	void poll_notify_one(px4_pollfd_struct_t *fds, pollevent_t events) override;

private:
	struct UpdateIntervalData {
		struct hrt_call update_call;  /**< deferred wakeup call if update_period is nonzero */
#ifndef __PX4_NUTTX
		uint64_t last_update; /**< time at which the last update was provided, used when update_interval is nonzero */
#endif
		unsigned  interval; /**< if nonzero minimum interval between updates */
		bool update_reported;
	};
	struct SubscriberData {
		~SubscriberData() { if (update_interval) { delete (update_interval); } }

		unsigned  generation; /**< last generation the subscriber has seen */
		UpdateIntervalData *update_interval; /**< if null, no update interval */

		// these flags are only used if update_interval != null
		bool update_reported() const { return update_interval ? update_interval->update_reported : false; }
		void set_update_reported(bool update_reported_flag)
		{ if (update_interval) { update_interval->update_reported = update_reported_flag; } }
	};

	const orb_metadata *_meta; /**< object metadata information */
	const uint8_t _instance; /**< orb multi instance identifier */
	uint8_t     *_data{nullptr};   /**< allocated object buffer */
	hrt_abstime   _last_update{0}; /**< time the object was last updated */
	volatile unsigned   _generation{0};  /**< object generation count */
	uint8_t   _priority;  /**< priority of the topic */
	bool _published{false};  /**< has ever data been published */
	uint8_t _queue_size; /**< maximum number of elements in the queue */
	int8_t _subscriber_count{0};

	px4_task_t _publisher{0}; /**< if nonzero, current publisher. Only used inside the advertise call.
						We allow one publisher to have an open file descriptor at the same time. */

	// statistics
	uint32_t _lost_messages = 0; /**< nr of lost messages for all subscribers. If two subscribers lose the same
					message, it is counted as two. */

	inline static SubscriberData    *filp_to_sd(cdev::file_t *filp);

	/**
	 * Perform a deferred update for a rate-limited subscriber.
	 */
	void      update_deferred();

	/**
	 * Bridge from hrt_call to update_deferred
	 *
	 * void *arg    ORBDevNode pointer for which the deferred update is performed.
	 */
	static void   update_deferred_trampoline(void *arg);

	/**
	 * Check whether a topic appears updated to a subscriber.
	 *
	 * Lock must already be held when calling this.
	 *
	 * @param sd    The subscriber for whom to check.
	 * @return    True if the topic should appear updated to the subscriber
	 */
	bool      appears_updated(SubscriberData *sd);

};
