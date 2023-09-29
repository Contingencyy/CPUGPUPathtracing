#pragma once

#include <utility>

static constexpr float PI = 3.14159265f;
static constexpr float INV_PI = 1.0f / PI;

inline float Deg2Rad(float deg)
{
	return deg * PI / 180.0f;
}

inline float Rad2Deg(float rad)
{
	return rad * 180.0f / PI;
}

struct Vec2
{
	Vec2() = default;
	Vec2(float _s)
		: x(_s), y(_s) {}
	Vec2(float _x, float _y)
		: x(_x), y(_y) {}

	union
	{
		struct
		{
			float x, y;
		};
		float xy[2] = { 0 };
	};
};

struct Vec3
{
	Vec3() = default;
	Vec3(float _s)
		: x(_s), y(_s), z(_s) {}
	Vec3(float _x, float _y, float _z)
		: x(_x), y(_y), z(_z) {}

	union
	{
		struct
		{
			float x, y, z;
		};
		float xyz[3] = { 0 };
	};
};

inline Vec3 operator+(const Vec3& v0, const Vec3& v1) { return Vec3(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z); }
inline Vec3 operator-(const Vec3& v0, const Vec3& v1) { return Vec3(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z); }
inline Vec3 operator*(const Vec3& v0, const float s) { return Vec3(v0.x * s, v0.y * s, v0.z * s); }
inline Vec3 operator*(const float s, const Vec3& v0) { return Vec3(v0.x * s, v0.y * s, v0.z * s); }

inline float Vec3Dot(const Vec3& v0, const Vec3& v1) { return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z; }
inline float Vec3Length(const Vec3& v0) { return Vec3Dot(v0, v0); }
inline Vec3 Vec3Normalize(const Vec3& v0) { float rcp_length = 1.0f / Vec3Length(v0); return v0 * rcp_length; }

struct Vec4
{
	Vec4() = default;
	Vec4(float _s)
		: x(_s), y(_s), z(_s), w(_s) {}
	Vec4(float _x, float _y, float _z, float _w)
		: x(_x), y(_y), z(_z), w(_w) {}
	
	union
	{
		struct
		{
			float x, y, z, w;
		};
		float xyzw[4] = { 0 };
	};
};

inline uint32_t Vec4ToUint(const Vec4& v0)
{
	uint8_t r = (uint8_t)(255.0f * std::min(1.0f, v0.x));
	uint8_t g = (uint8_t)(255.0f * std::min(1.0f, v0.y));
	uint8_t b = (uint8_t)(255.0f * std::min(1.0f, v0.z));
	uint8_t a = (uint8_t)(255);

	return (a << 24) + (b << 16) + (g << 8) + r;
}