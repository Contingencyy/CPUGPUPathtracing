#pragma once
#include "MathLib.h"

#define INTERSECTION_SPHERE_GEOMETRIC 1
#define INTERSECTION_SPHERE_QUADRATIC 0

#define INTERSECTION_TRIANGLE_INSIDE_OUTSIDE_TEST 0
#define INTERSECTION_TRIANGLE_MOELLER_TRUMBORE 1

#include <vector>

struct Vertex
{
	Vec3 pos;
	Vec3 normal;
};

enum PrimitiveType : uint8_t
{
	PrimitiveType_Plane,
	PrimitiveType_Sphere,
	PrimitiveType_Triangle,
	PrimitiveType_AABB,
	PrimitiveType_NumTypes
};

struct Mesh
{
	std::vector<Vertex> vertices;
	std::vector<uint32_t> indices;
};

struct Primitive
{
	PrimitiveType type = PrimitiveType_NumTypes;
	void* prim_ptr = nullptr;
	uint32_t mat_index = 0;
};

struct Plane
{
	Vec3 normal = Vec3(0.0f);
	Vec3 point = Vec3(0.0f);
};

struct Sphere
{
	Vec3 center = Vec3(0.0f);
	float radius_sq = 0.0f;
};

struct Triangle
{
	Vertex v0;
	Vertex v1;
	Vertex v2;
};

struct AABB
{
	Vec3 pmin = Vec3(0.0f);
	Vec3 pmax = Vec3(0.0f);
};

struct Ray
{
	Ray(const Vec3& orig, const Vec3& dir)
		: origin(orig), direction(dir)
	{
		inv_direction = 1.0f / dir;
	}

	Vec3 origin = Vec3(0.0f);
	Vec3 direction = Vec3(0.0f);
	Vec3 inv_direction = Vec3(0.0f);
	float t = 1e34f;

	struct Payload
	{
		uint32_t obj_idx = ~0u;
		uint32_t tri_idx = 0;
		uint32_t bvh_depth = 0;
	} payload;
};

float IntersectAABB(const AABB& aabb, Ray& ray);
float GetAABBVolume(const AABB& aabb);
void GrowAABB(AABB& aabb, const Vec3& p);

bool IntersectPlane(const Plane& plane, Ray& ray);

bool IntersectSphere(const Sphere& sphere, Ray& ray);
Vec3 GetSphereNormalAtPoint(const Sphere& sphere, const Vec3& point);
Vec3 GetSphereCentroid(const Sphere& sphere);
AABB GetSphereBounds(const Sphere& sphere);

bool IntersectTriangle(const Triangle& triangle, Ray& ray);
Vec3 GetTriangleNormalAtPoint(const Triangle& triangle, const Vec3& point);
Vec3 GetTriangleCentroid(const Triangle& triangle);
AABB GetTriangleBounds(const Triangle& triangle);
