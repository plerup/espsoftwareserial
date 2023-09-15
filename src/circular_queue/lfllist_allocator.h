/*
 lfllist_allocator.h
 A lock free double-linked list based allocator.
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

#ifndef __LFLLIST_ALLOCATOR_H
#define __LFLLIST_ALLOCATOR_H

#include "lfllist.h"

namespace ghostl
{
    template<typename T, std::size_t CAPACITY>
    struct lfllist_allocator : private lfllist<T>
    {
        // type definitions
        using lfllist = lfllist<T>;
        using node_type = lfllist::node_type;
        using value_type = T;
        using pointer = T*;
        using const_pointer = const T*;
        using reference = T&;
        using const_reference = const T&;
        using size_type = std::size_t;
        using difference_type = std::ptrdiff_t;

        lfllist_allocator()
        {
            for (size_type i = 0; i < CAPACITY; ++i)
            {
                auto node = reinterpret_cast<node_type*>(&span[i * sizeof(node_type)]);
                node = new (node) node_type();
                lfllist::push(node);
            }
        }
        lfllist_allocator(const lfllist_allocator&) = delete;
        lfllist_allocator(lfllist_allocator&&) = delete;
        ~lfllist_allocator()
        {
            node_type* node;
            while (nullptr != (node = lfllist::back()))
            {
                lfllist::remove(node);
            }
        }
        auto operator =(const lfllist_allocator&)->lfllist_allocator & = delete;
        auto operator =(lfllist_allocator&&)->lfllist_allocator & = delete;
        [[nodiscard]] pointer allocate(size_type n, const void* = nullptr)
        {
            if (n != 1)
            {
                return nullptr;
            }
            node_type* node;
            if (!lfllist::try_pop(node))
            {
                return nullptr;
            }
            return reinterpret_cast<pointer>(node);
        }
        constexpr void deallocate(pointer const p, size_t n)
        {
            auto node = reinterpret_cast<node_type*>(p);
            lfllist::push(node);
        }
    private:
        char span[CAPACITY * sizeof(node_type)] = { 0 };
    };
}

#endif // __LFLLIST_ALLOCATOR_H
