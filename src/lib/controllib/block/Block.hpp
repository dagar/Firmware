/****************************************************************************
 *
 *   Copyright (C) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file Block.hpp
 *
 * Controller library code
 */

#pragma once

#include <containers/IntrusiveList.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <controllib/block/BlockParam.hpp>

namespace control
{

static constexpr uint8_t maxChildrenPerBlock = 100;
static constexpr uint8_t maxParamsPerBlock = 110;
static constexpr uint8_t maxSubscriptionsPerBlock = 100;
static constexpr uint8_t maxPublicationsPerBlock = 100;
static constexpr uint8_t blockNameLengthMax = 40;

// forward declaration
class BlockParamBase;
class SuperBlock;

/**
 */
class __EXPORT Block : public IntrusiveListNode<Block *>
{
public:
	friend class BlockParamBase;

	Block(SuperBlock *parent, const char *name);
	virtual ~Block() = default;

	// no copy, assignment, move, move assignment
	Block(const Block &) = delete;
	Block &operator=(const Block &) = delete;
	Block(Block &&) = delete;
	Block &operator=(Block &&) = delete;

	void getName(char *name, size_t n);

	virtual void updateParams();
	virtual void updateSubscriptions();
	virtual void updatePublications();

	virtual void setDt(float dt) { _dt = dt; }
	float getDt() { return _dt; }

protected:

	virtual void updateParamsSubclass() {}

	SuperBlock *getParent() { return _parent; }
	IntrusiveList<uORB::SubscriptionNode *> &getSubscriptions() { return _subscriptions; }
	IntrusiveList<uORB::PublicationNode *> &getPublications() { return _publications; }
	IntrusiveList<BlockParamBase *> &getParams() { return _params; }

	const char *_name;
	SuperBlock *_parent;
	float _dt{0.0f};

	IntrusiveList<uORB::SubscriptionNode *> _subscriptions;
	IntrusiveList<uORB::PublicationNode *> _publications;
	IntrusiveList<BlockParamBase *> _params;
};

class __EXPORT SuperBlock :
	public Block
{
public:
	friend class Block;

	SuperBlock(SuperBlock *parent, const char *name) : Block(parent, name) {}
	~SuperBlock() = default;

	// no copy, assignment, move, move assignment
	SuperBlock(const SuperBlock &) = delete;
	SuperBlock &operator=(const SuperBlock &) = delete;
	SuperBlock(SuperBlock &&) = delete;
	SuperBlock &operator=(SuperBlock &&) = delete;

	void setDt(float dt) override;

	void updateParams() override
	{
		Block::updateParams();

		if (getChildren().front() != nullptr) { updateChildParams(); }
	}

	void updateSubscriptions() override
	{
		Block::updateSubscriptions();

		if (getChildren().front() != nullptr) { updateChildSubscriptions(); }
	}
	void updatePublications() override
	{
		Block::updatePublications();

		if (getChildren().front() != nullptr) { updateChildPublications(); }
	}

protected:
	IntrusiveList<Block *> &getChildren() { return _children; }
	void updateChildParams();
	void updateChildSubscriptions();
	void updateChildPublications();

	IntrusiveList<Block *> _children;
};


} // namespace control
