#pragma once
#include "MathLib.h"

static uint32_t s_seed = 0x12345678;

inline uint32_t WangHash(uint32_t seed)
{
	seed = (seed ^ 61) ^ (seed >> 16);
	seed *= 9, seed = seed ^ (seed >> 4);
	seed *= 0x27d4eb2d;
	seed = seed ^ (seed >> 15);
	return seed;
}

inline uint32_t RandomUInt32()
{
	s_seed ^= s_seed << 13;
	s_seed ^= s_seed >> 17;
	s_seed ^= s_seed << 5;
	return s_seed;
}

inline float RandomFloat()
{
	return RandomUInt32() * 2.3283064365387e-10f;
}

inline float RandomFloatRange(float min = 0.0f, float max = 1.0f)
{
	return min + (RandomFloat() * (max - min));
}

inline Vec3 RandomVec3(float min = 0.0f, float max = 1.0f)
{
	return Vec3(RandomFloatRange(min, max), RandomFloatRange(min, max), RandomFloatRange(min, max));
}
