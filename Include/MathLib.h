#pragma once

#include <cmath>

static constexpr float PI = 3.14159265f;
static constexpr float TWO_PI = 2.0f * PI;
static constexpr float INV_PI = 1.0f / PI;

inline constexpr float Deg2Rad(float deg)
{
	return deg * PI / 180.0f;
}

inline constexpr float Rad2Deg(float rad)
{
	return rad * 180.0f / PI;
}

bool SolveQuadratic(const float a, const float b, const float c, float& x0, float& x1);

struct UVec2
{
	UVec2() = default;
	UVec2(uint32_t _s)
		: x(_s), y(_s) {}
	UVec2(uint32_t _x, uint32_t _y)
		: x(_x), y(_y) {}

	union
	{
		struct
		{
			uint32_t x, y;
		};
		uint32_t xy[2] = { 0 };
	};
};

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

	inline Vec3& operator+=(const Vec3& v0) { x += v0.x; y += v0.y; z += v0.z; return *this; }
	inline Vec3& operator-=(const Vec3& v0) { x -= v0.x; y -= v0.y; z -= v0.z; return *this; }
	inline Vec3& operator*=(const Vec3& v0) { x *= v0.x; y *= v0.y; z *= v0.z; return *this; }
	inline Vec3& operator/=(const Vec3& v0) { x /= v0.x; y /= v0.y; z /= v0.z; return *this; }
};

inline Vec3 operator-(const Vec3& v0) { return Vec3(-v0.x, -v0.y, -v0.z); }

inline Vec3 operator+(const Vec3& v0, const Vec3& v1) { return Vec3(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z); }
inline Vec3 operator-(const Vec3& v0, const Vec3& v1) { return Vec3(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z); }
inline Vec3 operator*(const Vec3& v0, const Vec3& v1) { return Vec3(v0.x * v1.x, v0.y * v1.y, v0.z * v1.z); }
inline Vec3 operator*(const Vec3& v0, const float s) { return Vec3(v0.x * s, v0.y * s, v0.z * s); }
inline Vec3 operator*(const float s, const Vec3& v0) { return Vec3(s * v0.x, s * v0.y, s * v0.z); }
inline Vec3 operator/(const Vec3& v0, const float s) { return Vec3(v0.x / s, v0.y / s, v0.z / s); }
inline Vec3 operator/(const float s, const Vec3& v0) { return Vec3(s / v0.x, s / v0.y, s / v0.z); }

inline Vec3 Vec3Cross(const Vec3& v0, const Vec3& v1) { return Vec3(v0.y * v1.z - v0.z * v1.y, v0.z * v1.x - v0.x * v1.z, v0.x * v1.y - v0.y * v1.x); }
inline float Vec3Dot(const Vec3& v0, const Vec3& v1) { return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z; }
inline float Vec3Length(const Vec3& v0) { return std::sqrtf(Vec3Dot(v0, v0)); }
inline Vec3 Vec3Normalize(const Vec3& v0) { float rcp_length = 1.0f / Vec3Length(v0); return v0 * rcp_length; }

inline Vec3 Vec3Min(const Vec3& v0, const Vec3& v1) { return Vec3(std::min(v0.x, v1.x), std::min(v0.y, v1.y), std::min(v0.z, v1.z)); }
inline Vec3 Vec3Max(const Vec3& v0, const Vec3& v1) { return Vec3(std::max(v0.x, v1.x), std::max(v0.y, v1.y), std::max(v0.z, v1.z)); }
inline Vec3 Vec3Abs(const Vec3& v0) { return Vec3(std::fabs(v0.x), std::fabs(v0.y), std::fabs(v0.z)); }

inline Vec3 Vec3Lerp(const Vec3& v0, const Vec3& v1, float s) { return Vec3(v0.x + (v1.x - v0.x) * s, v0.y + (v1.y - v0.y) * s, v0.z + (v1.z - v0.z) * s); }
inline Vec3 Vec3Lerp(const Vec3& v0, const Vec3& v1, const Vec3& lerp) { return Vec3(std::lerp(v0.x, v1.x, lerp.x), std::lerp(v0.y, v1.y, lerp.y), std::lerp(v0.z, v1.z, lerp.z)); }
inline Vec3 Vec3Clamp(const Vec3& v0, float min, float max) { return Vec3(std::clamp(v0.x, min, max), std::clamp(v0.y, min, max), std::clamp(v0.z, min, max)); }
inline Vec3 Vec3Pow(const Vec3& v0, float pow) { return Vec3(std::pow(v0.x, pow), std::pow(v0.y, pow), std::pow(v0.z, pow)); }

struct Vec4
{
	Vec4() = default;
	Vec4(float _s)
		: x(_s), y(_s), z(_s), w(_s) {}
	Vec4(float _x, float _y, float _z, float _w)
		: x(_x), y(_y), z(_z), w(_w) {}
	Vec4(const Vec3& xyz, float w)
		: x(xyz.x), y(xyz.y), z(xyz.z), w(w) {}
	
	union
	{
		struct
		{
			float x, y, z, w;
		};
		float xyzw[4] = { 0 };
		struct
		{
			Vec3 xyz;
			float w;
		};
	};

	inline Vec4& operator+=(const Vec4& v0) { x += v0.x; y += v0.y; z += v0.z; w += v0.w; return *this; }
	inline Vec4& operator-=(const Vec4& v0) { x -= v0.x; y -= v0.y; z -= v0.z; w -= v0.w; return *this; }
	inline Vec4& operator*=(const Vec4& v0) { x *= v0.x; y *= v0.y; z *= v0.z; w *= v0.w; return *this; }
	inline Vec4& operator/=(const Vec4& v0) { x /= v0.x; y /= v0.y; z /= v0.z; w /= v0.w; return *this; }
};

inline Vec4 operator-(const Vec4& v0) { return Vec4(-v0.x, -v0.y, -v0.z, -v0.w); }

inline Vec4 operator+(const Vec4& v0, const Vec4& v1) { return Vec4(v0.x + v1.x, v0.y + v1.y, v0.z + v1.z, v0.w + v1.w); }
inline Vec4 operator-(const Vec4& v0, const Vec4& v1) { return Vec4(v0.x - v1.x, v0.y - v1.y, v0.z - v1.z, v0.w - v1.w); }
inline Vec4 operator*(const Vec4& v0, const Vec4& v1) { return Vec4(v0.x * v1.x, v0.y * v1.y, v0.z * v1.z, v0.w * v1.w); }
inline Vec4 operator*(const Vec4& v0, const float s) { return Vec4(v0.x * s, v0.y * s, v0.z * s, v0.w * s); }
inline Vec4 operator*(const float s, const Vec4& v0) { return Vec4(v0.x * s, v0.y * s, v0.z * s, v0.w * s); }
inline Vec4 operator/(const Vec4& v0, const float s) { return Vec4(v0.x / s, v0.y / s, v0.z / s, v0.w / s); }
inline Vec4 operator/(const float s, const Vec4& v0) { return Vec4(v0.x / s, v0.y / s, v0.z / s, v0.w / s); }

inline uint32_t Vec4ToUint(const Vec4& v0)
{
	uint8_t r = (uint8_t)(255.0f * std::min(1.0f, v0.x));
	uint8_t g = (uint8_t)(255.0f * std::min(1.0f, v0.y));
	uint8_t b = (uint8_t)(255.0f * std::min(1.0f, v0.z));
	uint8_t a = (uint8_t)(255);

	return (a << 24) + (b << 16) + (g << 8) + r;
}
