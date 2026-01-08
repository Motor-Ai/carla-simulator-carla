
// Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB).
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#define MSG_STRINGIFY_HELPER(x) #x
#define MSG_STRINGIFY(x) MSG_STRINGIFY_HELPER(x)

// The logic to decide the extension
#define USE_HPP_EXT
#if defined(USE_HPP_EXT)
    #define MSG_EXT(f) MSG_STRINGIFY(f.hpp)
#else
    #define MSG_EXT(f) MSG_STRINGIFY(f.h)
#endif
