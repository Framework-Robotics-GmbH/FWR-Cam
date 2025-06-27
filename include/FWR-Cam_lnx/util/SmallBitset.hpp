//------------------------------------------------------------------------------
// FWR-Cam Library
// Copyright (c) 2025 Framework Robotics GmbH
// 
// Authors: Michael Lampert
// 
// Licensed under the BSD 3-Clause License.
// You may use, modify, and redistribute this file under those terms.
// 
// See the LICENSE file in the project root for full license information.
//------------------------------------------------------------------------------

#pragma once

#include <cstdint>
#include <cstddef>
#include <type_traits>
#include <bit>






template <size_t N>
class SmallBitset
{
    static_assert(N > 0 && N <= sizeof(size_t), "Only supports 1 <= N <= 64");
public:
    using value_t = std::conditional_t
                    < (N <= 8)
                    , uint8_t
                    , std::conditional_t
                      < (N <= 16), uint16_t
                      , std::conditional_t
                        < (N <= 32)
                        , uint32_t
                        , uint64_t
                        >
                      >
                    >;
    
    constexpr void   set(size_t pos) noexcept { bits |=   value_t(1) << pos ; }
    constexpr void reset(size_t pos) noexcept { bits &= ~(value_t(1) << pos); }
    constexpr void clear()           noexcept { bits  = 0                ; }
    
    constexpr bool test(size_t pos) const noexcept { return (bits >> pos) & 1; }
    
    constexpr bool operator[](size_t pos) const noexcept { return test(pos); }
    
    constexpr value_t raw(         ) const noexcept { return bits; }
    constexpr void    raw(value_t v)       noexcept { bits = v   ; }
    
    constexpr size_t count() const noexcept { return std::popcount(bits); }
    
private:
    value_t bits{};
};
