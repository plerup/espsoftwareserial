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
    template<typename T, typename ForEachArg = void>
    struct lfllist
    {
        lfllist() = default;
        lfllist(const lfllist&) = delete;
        lfllist(lfllist&&) = delete;
        ~lfllist()
        {
            while (auto node = back()) erase(node);
        }
        auto operator =(const lfllist&)->lfllist & = delete;
        auto operator =(lfllist&&)->lfllist & = delete;

        struct node_type
        {
        private:
            friend lfllist;
            std::atomic<node_type*> pred{ nullptr };
            std::atomic<node_type*> next{ nullptr };
            std::atomic<bool> erase_lock{ false };
        public:
            T item;
        };

        /// <summary>
        ///  Emplace an item at the list's front. Is safe for concurrency and reentrance.
        /// </summary>
        /// <param name="toInsert">The item to emplace.</param>
        /// <returns>The pointer to new node, nullptr on failure.</returns>
        [[nodiscard]] auto emplace_front(T&& toInsert) -> node_type*
        {
            auto node = new (std::nothrow) node_type();
            if (!node) return nullptr;
            node->item = std::move(toInsert);
            std::atomic_thread_fence(std::memory_order_release);

            auto next = first.exchange(node);
            node->next.store(next);
            std::atomic_thread_fence(std::memory_order_release);
            next->pred.store(node);
            std::atomic_thread_fence(std::memory_order_release);

            return node;
        };

        /// <summary>
        /// Erase a previously emplaced node from the list.
        /// Using erase(), full concurrency safety for unique nodes.
        /// Non-reentrant, non-concurrent with for_each().
        /// </summary>
        /// <param name="to_erase">An item (not nullptr) that must be a member of this list.</param>
        auto erase(node_type* const to_erase) -> void
        {
            node_type* next = nullptr;
            node_type* pred = nullptr;
            while (to_erase->erase_lock.exchange(true)) {}
            for (;;)
            {
                next = to_erase->next.load();
                if (!next->erase_lock.exchange(true))
                {
                    pred = to_erase->pred.load();
                    if (next == to_erase->next.load()) break;
                    next->erase_lock.store(false);
                }
            }
            for (;;)
            {
                do
                {
                    next->pred.store(pred);
                    if (pred) pred->next.store(next);
                } while (!to_erase->pred.compare_exchange_weak(pred, pred));
                auto _to_erase = to_erase;
                if (pred || first.compare_exchange_strong(_to_erase, next)) break;
                pred = to_erase->pred.load();
            }
            next->erase_lock.store(false);
            delete(to_erase);
        };

        [[nodiscard]] auto front() -> node_type*
        {
            return first.load();
        };

        [[nodiscard]] auto back() -> node_type*
        {
            return last_sentinel.pred.load();
        };

        /// <summary>
        /// Traverse every node of this list in FIFO, then erase that node.
        /// The ownership of the item contained in that node is passed to parameter function.
        /// Non-reentrant, non-concurrent with erase() and for_each().
        /// </summary>
        /// <param name="to_erase">A function this is invoked for each node of this list.</param>
#if defined(ESP8266) || defined(ESP32) || !defined(ARDUINO)
        void for_each(const Delegate<void(T&&), ForEachArg>& fn)
#else
        void for_each(Delegate<void(T&&), ForEachArg> fn)
#endif
        {
            while (auto node = back())
            {
                fn(std::move(std::exchange(node->item, {})));
                erase(node);
            }
        };

    private:
        node_type last_sentinel;
        std::atomic<node_type*> first = &last_sentinel;
    };
}

#endif // __LFLLIST_H
