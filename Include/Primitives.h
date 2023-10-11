#pragma once
#include "MathLib.h"

#define INTERSECTION_SPHERE_GEOMETRIC 1
#define INTERSECTION_SPHERE_QUADRATIC 0

#define INTERSECTION_TRIANGLE_INSIDE_OUTSIDE_TEST 0
#define INTERSECTION_TRIANGLE_MOELLER_TRUMBORE 1

enum PrimitiveType : uint8_t
{
	PrimitiveType_Plane,
	PrimitiveType_Sphere,
	PrimitiveType_Triangle,
	PrimitiveType_NumTypes
};

struct Plane
{
	Vec3 normal = Vec3(0.0f);
	Vec3 point = Vec3(0.0f);

	uint32_t mat_index = 0;
};

struct Sphere
{
	Vec3 center = Vec3(0.0f);
	float radius_sq = 0.0f;

	uint32_t mat_index = 0;
};

struct Triangle
{
	Vec3 v0 = Vec3(0.0f);
	Vec3 v1 = Vec3(0.0f);
	Vec3 v2 = Vec3(0.0f);
	Vec3 centroid = Vec3(0.0f);

	uint32_t mat_index = 0;
};

struct AABB
{
	Vec3 pmin = Vec3(0.0f);
	Vec3 pmax = Vec3(0.0f);
};

struct Ray
{
	Vec3 origin = Vec3(0.0f);
	Vec3 direction = Vec3(0.0f);
	float t = 1e34f;

	struct Payload
	{
		PrimitiveType object_type = PrimitiveType_NumTypes;
		void* object_ptr = nullptr;
		uint32_t mat_index = 0;
		uint32_t bvh_depth = 0;
	} payload;
};

bool IntersectAABB(const AABB& aabb, Ray& ray);
void IntersectPlane(const Plane& plane, Ray& ray);
void IntersectSphere(const Sphere& sphere, Ray& ray);
void IntersectTriangle(const Triangle& triangle, Ray& ray);
Vec3 GetObjectSurfaceNormalAtPoint(PrimitiveType type, void* ptr, const Vec3& point);
