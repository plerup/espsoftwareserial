/*
 lfllist.h
 A lock-free double-linked list implementation.
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
            while (auto node = first.load()) erase(node);
        }
        auto operator =(const lfllist&) -> lfllist& = delete;
        auto operator =(lfllist&&) -> lfllist& = delete;
        struct node_type
        {
            std::atomic<node_type*> pred{ nullptr };
            std::atomic<node_type*> next{ nullptr };
            T item;
        };
        
        std::atomic<node_type*> first = nullptr;

        /// <summary>
        ///  Emplace an item at the list's front. Is safe for concurrence and reentrance.
        /// </summary>
        /// <param name="toInsert">The item to emplace.</param>
        /// <returns>The pointer to new node, nullptr on failure.</returns>
        [[nodiscard]] auto emplace_front(T&& toInsert) -> node_type*
        {
            auto node = new (std::nothrow) node_type();
            if (!node) return nullptr;
        
            node->item = std::move(toInsert);
        
            auto _first = first.load();
            do
            {
                node->next.store(_first);
            }
            while (!first.compare_exchange_weak(_first, node));
            if (_first) _first->pred.store(node);
            return node;
        };

        /// <summary>
        /// Erase a previously emplaced node from the list.
        /// Non-reentrant, non-concurrent: erase(), for_each().
        /// </summary>
        /// <param name="to_erase">An item that must be a member of this list.</param>
        auto erase(node_type* const to_erase) -> void
        {
            for (;;)
            {
                auto pred = to_erase->pred.load();
                auto next = to_erase->next.load();
                if (pred)
                {
                    pred->next.store(next);
                    next->pred.store(pred);
                }
                else
                {
                    auto current = to_erase;
                    if (!first.compare_exchange_weak(current, next)) continue;
                    current = to_erase;
                    if (next && !next->pred.compare_exchange_weak(current, pred)) continue;
                }
        
                delete(to_erase);
                break;
            }
        };

        /// <summary>
        /// Traverse every node of this list, then erase that node.
        /// The ownership of the item contained in that node is passed to parameter function.
        /// Non-reentrant, non-concurrent: erase(), for_each().
        /// </summary>
        /// <param name="to_erase">A function this is invoked for each node of this list.</param>
#if defined(ESP8266) || defined(ESP32) || !defined(ARDUINO)
        void for_each(const Delegate<void(T&&), ForEachArg>& fn)
#else
        void for_each(Delegate<void(T&&), ForEachArg> fn)
#endif
        {
            while (auto node = first.load())
            {
                fn(std::move(std::exchange(node->item, {})));
                erase(node);
            }
        };
    };
}

#endif // __LFLLIST_H
