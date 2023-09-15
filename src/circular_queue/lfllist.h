/*
 lfllist.h
 A lock free double-linked list implementation.
 Copyright (c) 2023 Dirk O. Kaar

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#pragma once

#ifndef __LFLLIST_H
#define __LFLLIST_H

#include "Delegate.h"

#include <atomic>
#include <utility>

namespace ghostl
{
    template<typename T, class Allocator, typename ForEachArg>
    struct lfllist;

    namespace detail {
        template<typename T>
        struct lfllist_node_type
        {
            T item; // must be first member to facilitate reinterpret_cast.
            using node_type = lfllist_node_type<T>;
            lfllist_node_type() = default;
            explicit lfllist_node_type(T&& _item) : item(std::move(_item)) {}
        private:
            template<typename, class, typename> friend struct ghostl::lfllist;
            std::atomic<node_type*> pred{ nullptr };
            std::atomic<node_type*> next{ nullptr };
            std::atomic<bool> remove_lock{ false };
        };
    };

    template<typename T, class Allocator = std::allocator<detail::lfllist_node_type<T>>, typename ForEachArg = void>
    struct lfllist
    {
        using node_type = detail::lfllist_node_type<T>;

        lfllist() = default;
        lfllist(const lfllist&) = delete;
        lfllist(lfllist&&) = delete;
        ~lfllist()
        {
            node_type* node;
            while (nullptr != (node = back())) erase(node);
        }
        auto operator =(const lfllist&)->lfllist & = delete;
        auto operator =(lfllist&&)->lfllist & = delete;

        /// <summary>
        ///  Emplace an item at the list's front. Is safe for concurrency and reentrance.
        /// </summary>
        /// <param name="toInsert">The item to emplace.</param>
        /// <returns>The pointer to new node, nullptr on failure.</returns>
        [[nodiscard]] auto IRAM_ATTR emplace_front(T&& toInsert) -> node_type*
        {
            auto node = std::allocator_traits<Allocator>::allocate(alloc, 1);
            if (!node) return nullptr;
            std::allocator_traits<Allocator>::construct(alloc, node, std::move(toInsert));
            std::atomic_thread_fence(std::memory_order_release);
            push(node);
            return node;
        };

        /// <summary>
        ///  Push a node to the list's front. Is safe for concurrency and reentrance.
        /// </summary>
        /// <param name="node">A node pointer from a prior remove().</param>
        /// <returns>, nullptr on failure.</returns>
        auto IRAM_ATTR push(node_type* const node) -> void
        {
            auto next = first.exchange(node);
            node->next.store(next);
            std::atomic_thread_fence(std::memory_order_release);
            next->pred.store(node);
            std::atomic_thread_fence(std::memory_order_release);
        };

        /// <summary>
        /// Remove, without destroying it, a member node from the list.
        /// Using remove() or erase(), full concurrency safety for unique nodes.
        /// Non-reentrant, non-concurrent with for_each(), and try_pop().
        /// Caveat: for_each() or try_pop() may erase any node pointer.
        /// </summary>
        /// <param name="node">A node (not nullptr) that must be a member of this list.</param>
        auto remove(node_type* const node) -> void
        {
            while (!try_remove(node)) {}
        }

        /// <summary>
        /// Try to remove, without destroying it, a member node from the list.
        /// Using remove() or erase(), full concurrency safety for unique nodes.
        /// Non-reentrant, non-concurrent with for_each(), and try_pop().
        /// Caveat: for_each() or try_pop() may erase any node pointer.
        /// </summary>
        /// <param name="node">A node (not nullptr) that must be a member of this list.</param>
        /// <returns>True on success, false if there is competition on locking node.</returns>
        auto try_remove(node_type* const node) -> bool
        {
            auto _false = false;
            if (!node->remove_lock.compare_exchange_strong(_false, true)) { return false; }
            node_type* next = nullptr;
            node_type* pred = nullptr;
            for (;;)
            {
                next = node->next.load();
                if (next->remove_lock.compare_exchange_strong(_false, true))
                {
                    pred = node->pred.load();
                    if (next == node->next.load()) break;
                    next->remove_lock.store(false);
                }
                else
                {
                    _false = false;
                }
            }
            for (;;)
            {
                next->pred.store(pred);
                if (pred) pred->next.store(next);
                auto _node = node;
                if (!pred && !first.compare_exchange_strong(_node, next))
                {
                    while (node->pred.compare_exchange_strong(pred, pred)) {}
                    continue;
                }
                break;
            }
            next->remove_lock.store(false);
            node->remove_lock.store(false);
            node->pred.store(nullptr);
            node->next.store(nullptr);
            return true;
        };

        /// <summary>
        /// Erase a previously emplaced node from the list.
        /// Using erase(), full concurrency safety for unique nodes.
        /// Non-reentrant, non-concurrent with for_each(), and try_pop().
        /// Caveat: for_each() or try_pop() may erase any node pointer.
        /// </summary>
        /// <param name="to_erase">An item (not nullptr) that must be a member of this list.</param>
        auto erase(node_type* const to_erase) -> void
        {
            while (!try_erase(to_erase)) {}
        }

        /// <summary>
        /// Try to erase a previously emplaced node from the list.
        /// Using try_erase(), full concurrency safety for unique nodes.
        /// Non-reentrant, non-concurrent with for_each(), and try_pop().
        /// Caveat: for_each() or try_pop() may erase any node pointer.
        /// </summary>
        /// <param name="to_erase">An item (not nullptr) that must be a member of this list.</param>
        /// <returns>True on success, false if there is competition on locking to_erase.</returns>
        auto try_erase(node_type* const to_erase) -> bool
        {
            if (!try_remove(to_erase)) return false;
            destroy(to_erase);
            return true;
        };

        auto destroy(node_type* const node) -> void
        {
            std::allocator_traits<Allocator>::destroy(alloc, node);
            std::allocator_traits<Allocator>::deallocate(alloc, node, 1);
        }

        [[nodiscard]] auto back() -> node_type*
        {
            if (auto node = last_sentinel.pred.load(); node) return node;
            return nullptr;
        }

        /// <summary>
        /// Try to atomically get the item of and erase a node at the back of the list.
        /// Using try_pop(), full concurrency safety.
        /// Non-reentrant, non-concurrent with erase().
        /// </summary>
        /// <param name="item">An out argument that on success, receives the item at the back of this list.</param>
        /// <returns>True on success, false if the queue is empty or there is competition.</returns>
        [[nodiscard]] auto try_pop(T& item) -> bool
        {
            node_type* node{};
            if (!try_pop(node)) return false;
            item = std::move(node->item);
            destroy(node);
            return true;
        };

        /// <summary>
        /// Try to atomically remove a node at the back of the list.
        /// Using try_pop(), full concurrency safety.
        /// Non-reentrant, non-concurrent with erase().
        /// </summary>
        /// <param name="item">An out argument that on success, receives the item at the back of this list.</param>
        /// <returns>True on success, false if the queue is empty or there is competition.</returns>
        [[nodiscard]] auto try_pop(node_type*& node) -> bool
        {
            auto _false = false;
            if (!pop_guard.compare_exchange_strong(_false, true)) { return false; }
            auto has_node = false;
            if (nullptr != (node = back()))
            {
                remove(node);
                has_node = true;
            }
            pop_guard.store(false);
            return has_node;
        };

        /// <summary>
        /// Traverse every node of this list in FIFO, then erase that node.
        /// The ownership of the item contained in that node is passed to parameter function.
        /// Non-reentrant, non-concurrent with erase(), try_pop(), and for_each().
        /// </summary>
        /// <param name="to_erase">A function this is invoked for each node of this list.</param>
#if defined(ESP8266) || defined(ESP32) || !defined(ARDUINO)
        void for_each(const Delegate<void(T&&), ForEachArg>& fn)
#else
        void for_each(Delegate<void(T&&), ForEachArg> fn)
#endif
        {
            while (back())
            {
                if (T item; try_pop(item)) fn(std::move(item));
            }
        };

    private:
        Allocator alloc;
        node_type last_sentinel;
        std::atomic<node_type*> first = &last_sentinel;
        std::atomic<bool> pop_guard{ false };
    };
}

#endif // __LFLLIST_H
