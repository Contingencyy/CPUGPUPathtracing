#pragma once
#include "MathLib.h"

#define USE_SSE_INSTRUCTION_SET_INTERSECTIONS 1

#if USE_SSE_INSTRUCTION_SET_INTERSECTIONS
#include <immintrin.h>
#endif

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
	Ray(const Vec3& orig, const Vec3& dir, float dist)
		: origin(orig), direction(dir), t(dist)
	{
		inv_direction = 1.0f / dir;
	}

	union { struct { Vec3 origin; float dummy; }; __m128 origin4 = {}; };
	union { struct { Vec3 direction; float dummy; }; __m128 direction4 = {}; };
	union { struct { Vec3 inv_direction; float dummy; }; __m128 inv_direction4 = {}; };
	float t = 1e34f;

	struct Payload
	{
		uint32_t obj_idx = ~0u;
		uint32_t tri_idx = 0;
		uint32_t bvh_depth = 0;
	} payload;
};

struct Primitive
{
	Primitive() = default;
	Primitive(const Triangle& triangle)
		: type(PrimitiveType_Triangle), triangle(triangle) {}
	Primitive(const Sphere& sphere)
		: type(PrimitiveType_Sphere), sphere(sphere) {}
	Primitive(const Plane& plane)
		: type(PrimitiveType_Plane), plane(plane) {}
	Primitive(const AABB& aabb)
		: type(PrimitiveType_AABB), aabb(aabb) {}

	bool Intersect(Ray& ray) const;
	Vec3 Normal(const Vec3& pos) const;
	Vec3 RandomPoint() const;
	AABB Bounds() const;
	Vec3 Centroid() const;

	void RenderImGui();

	PrimitiveType type;

	union
	{
		Triangle triangle = {};
		Sphere sphere;
		Plane plane;
		AABB aabb;
	};
};

/*
	
	Intersection functions

*/
bool IntersectTriangle(const Triangle& triangle, Ray& ray);
bool IntersectSphere(const Sphere& sphere, Ray& ray);
bool IntersectPlane(const Plane& plane, Ray& ray);
float IntersectAABB_SSE(const __m128 aabb_min, const __m128 aabb_max, Ray& ray);
float IntersectAABB(const Vec3& aabb_min, const Vec3& aabb_max, Ray& ray);

/*

	Get normal at point on primitive functions

*/
Vec3 TriangleNormal(const Triangle& triangle, const Vec3& pos);
Vec3 SphereNormal(const Sphere& sphere, const Vec3& pos);
Vec3 PlaneNormal(const Plane& plane, const Vec3& pos);
Vec3 AABBNormal(const AABB& aabb, const Vec3& pos);

/*

	Generate random points on primitive functions

*/
Vec3 RandomPointTriangle(const Triangle& triangle);
Vec3 RandomPointSphere(const Sphere& sphere);
Vec3 RandomPointPlane(const Plane& plane);
Vec3 RandomPointAABB(const AABB& aabb);

/*

	Get primitive bounds

*/
AABB TriangleBounds(const Triangle& triangle);
AABB SphereBounds(const Sphere& sphere);
// Plane bounds makes no sense, its infinite. AABB bounds.. well, its already its own axis-aligned bounding box
//AABB PlaneBounds(const Plane& plane);
//AABB AABBBounds(const AABB& aabb);

/*

	Get primitive centroids

*/
Vec3 TriangleCentroid(const Triangle& triangle);
Vec3 SphereCentroid(const Sphere& sphere);
Vec3 AABBCentroid(const AABB& aabb);

/*

	Temporary functions

*/
float GetSphereSolidAngle(const Sphere& sphere, float dist_sq);
float GetTriangleArea(const Triangle& triangle);

/*

	Additional AABB functions

*/
float GetAABBVolume(const Vec3& aabb_min, const Vec3& aabb_max);
void GrowAABB(Vec3& aabb_min, Vec3& aabb_max, const Vec3& p);
