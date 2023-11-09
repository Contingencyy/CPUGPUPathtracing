#pragma once

#include "Logger.h"

#define ALIGN_POW2(x, align) ((intptr_t)(x) + ((align) - 1) & (-(intptr_t)(align)))
#define ALIGN_DOWN_POW2(x, align) ((intptr_t)(x) & (-(intptr_t)(align)))

#define ASSERT(x) assert(x)
#define EXCEPT(...) auto logged_msg = LOG_ERR(__VA_ARGS__); throw std::runtime_error(logged_msg)

#include <vector>
#include <array>
#include <string>
#include <stdint.h>
#include <chrono>
#include <unordered_map>
#include <algorithm>
#include <utility>

#include "MathLib.h"
#include "Random.h"
#include "Util.h"
