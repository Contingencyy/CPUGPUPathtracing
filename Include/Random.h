#pragma once
#include "MathLib.h"

#include <random>

inline float RandomFloat(float min = 0.0f, float max = 1.0f)
{
	return min + (float)(std::rand() / (RAND_MAX / (max - min)));
}

inline int32_t RandomInt32(int32_t min, int32_t max)
{
	return min + (std::rand() % (max + 1 - min));
}

inline uint32_t RandomUInt32(uint32_t min, uint32_t max)
{
	return min + (std::rand() % (max + 1 - min));
}

inline Vec3 RandomVec3(float min = 0.0f, float max = 1.0f)
{
	return Vec3(RandomFloat(min, max), RandomFloat(min, max), RandomFloat(min, max));
}
