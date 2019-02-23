/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file List.hpp
 *
 * A linked list.
 */

#pragma once

#include <stdlib.h>

template<typename T>
class ListNode
{
public:

	template<typename ...Args>
	ListNode(Args &&... args) : _data(args...) {}

	ListNode<T> *next() const { return _next; }
	ListNode<T> *prev() const { return _prev; }

	void setNext(ListNode<T> *next) { _next = next; }
	void setPrev(ListNode<T> *prev) { _prev = prev; }

	// join prev and next
	void remove()
	{
		if (_prev) {
			_prev->setNext(_next);
			_prev = nullptr;
		}

		if (_next) {
			_next->setPrev(_prev);
			_next = nullptr;
		}
	}

	//T *data() { return &_data; }
	T &data() { return _data; }

	//operator T &() { return _data; }

private:

	T _data;

	ListNode<T> *_next{nullptr};
	ListNode<T> *_prev{nullptr};
};

template<typename T>
class List
{
public:

	List() = default;
	~List() { clear(); }

	class Iterator
	{
	public:
		explicit Iterator(ListNode<T> *node): _node(node) {}
		Iterator() = delete;

		T *operator*() { return &_node->data(); }

		bool operator!=(const Iterator &it) const { return value() != it.value(); }

		Iterator &operator++()
		{
			_node = _node->next();
			return *this;
		}

		const ListNode<T> *value() const { return _node; }

	private:
		ListNode<T> *_node;
	};

	Iterator begin() { return Iterator{front()}; }
	Iterator end() { return Iterator{back()}; }

	template<typename ...Args>
	T emplace_front(Args &&... args)
	{
		ListNode<T> *newNode = new ListNode<T>(args...);
		newNode->setNext(front());
		newNode->setPrev(nullptr);
		_head = newNode;

		return newNode->data();
	}

	template<typename ...Args>
	T emplace_back(Args &&... args)
	{
		ListNode<T> *newNode = new ListNode<T>(args...);
		newNode->setNext(nullptr);
		newNode->setPrev(back());
		_tail = newNode;

		return newNode->data();
	}

	void pop_back()
	{
		if (_tail != nullptr) {
			ListNode<T> *oldTail = _tail;
			_tail = _tail->prev();
			_tail->setNext(nullptr);
			delete oldTail;
		}
	}

	void removeNode(ListNode<T> &node)
	{
		if (_head == node) {
			_head = _head->next();
		}

		if (_tail == node) {
			_tail = _tail->prev();
		}

		node.remove();
	}

	void deleteNode(ListNode<T> *node)
	{
		removeNode(node);
		delete node;
	}

	bool empty() const { return _head == nullptr; }

	size_t size()
	{
		size_t i = 0;

		for (const auto node : *this) {
			(void)node; // unused
			i++;
		}

		return i;
	}

	void clear()
	{
		const size_t sz = size();

		ListNode<T> *node = _head;

		for (size_t i = 0; i < sz; i++) {
			ListNode<T> *next = node->next();
			delete node;
			node = next;
		}

		_head = nullptr;
		_tail = nullptr;
	}

	ListNode<T> *front() const { return _head; }
	ListNode<T> *back() const { return _tail; }

protected:

	ListNode<T> *_head{nullptr};
	ListNode<T> *_tail{nullptr};

};
