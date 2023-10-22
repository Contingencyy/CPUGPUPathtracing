#include "Common.h"
#include "Primitives.h"

float IntersectAABB_SSE(const __m128 aabb_min, const __m128 aabb_max, Ray& ray)
{
	static __m128 mask4 = _mm_cmpeq_ps(_mm_setzero_ps(), _mm_set_ps(1, 0, 0, 0));
	__m128 t1 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(aabb_min, mask4), ray.origin4), ray.inv_direction4);
	__m128 t2 = _mm_mul_ps(_mm_sub_ps(_mm_and_ps(aabb_max, mask4), ray.origin4), ray.inv_direction4);
	__m128 vmax4 = _mm_max_ps(t1, t2), vmin4 = _mm_min_ps(t1, t2);
	float tmax = std::min(vmax4.m128_f32[0], std::min(vmax4.m128_f32[1], vmax4.m128_f32[2]));
	float tmin = std::max(vmin4.m128_f32[0], std::max(vmin4.m128_f32[1], vmin4.m128_f32[2]));
	
	if (tmax >= tmin && tmin < ray.t && tmax > 0.0f)
	{
		return tmin;
	}
	return 1e30f;
}

float IntersectAABB(const Vec3& aabb_min, const Vec3& aabb_max, Ray& ray)
{
	float tx1 = (aabb_min.x - ray.origin.x) * ray.inv_direction.x, tx2 = (aabb_max.x - ray.origin.x) * ray.inv_direction.x;
	float tmin = std::min(tx1, tx2), tmax = std::max(tx1, tx2);
	float ty1 = (aabb_min.y - ray.origin.y) * ray.inv_direction.y, ty2 = (aabb_max.y - ray.origin.y) * ray.inv_direction.y;
	tmin = std::max(tmin, std::min(ty1, ty2)), tmax = std::min(tmax, std::max(ty1, ty2));
	float tz1 = (aabb_min.z - ray.origin.z) * ray.inv_direction.z, tz2 = (aabb_max.z - ray.origin.z) * ray.inv_direction.z;
	tmin = std::max(tmin, std::min(tz1, tz2)), tmax = std::min(tmax, std::max(tz1, tz2));

	if (tmax >= tmin && tmin < ray.t && tmax > 0.0f)
	{
		return tmin;
	}
	return 1e30f;
}

float GetAABBVolume(const Vec3& aabb_min, const Vec3& aabb_max)
{
	Vec3 extent = aabb_max - aabb_min;
	return extent.x * extent.y + extent.y * extent.z + extent.z * extent.x;
}

void GrowAABB(Vec3& aabb_min, Vec3& aabb_max, const Vec3& p)
{
	aabb_min = Vec3Min(aabb_min, p);
	aabb_max = Vec3Max(aabb_max, p);
}

bool IntersectPlane(const Plane& plane, Ray& ray)
{
	// Plane: P * N + d = 0
	// Ray: P(t) = O + tD

	float denom = Vec3Dot(ray.direction, plane.normal);

	if (std::fabs(denom) > 1e-6)
	{
		Vec3 p0 = plane.point - ray.origin;
		float t = Vec3Dot(p0, plane.normal) / denom;

		if (t > 0.0f && t < ray.t)
		{
			ray.t = t;
			return true;
		}
	}

	return false;
}

bool IntersectSphere(const Sphere& sphere, Ray& ray)
{
	float t0, t1;
#if INTERSECTION_SPHERE_GEOMETRIC
	// Seems to be faster than solving the quadratic
	Vec3 L = sphere.center - ray.origin;
	float tca = Vec3Dot(L, ray.direction);

	if (tca < 0.0f)
	{
		return false;
	}

	float d2 = Vec3Dot(L, L) - tca * tca;

	if (d2 > sphere.radius_sq)
	{
		return false;
	}

	float thc = std::sqrtf(sphere.radius_sq - d2);
	t0 = tca - thc;
	t1 = tca + thc;
#elif INTERSECTION_SPHERE_QUADRATIC
	Vec3 L = ray.origin - sphere.center;
	float a = Vec3Dot(ray.direction, ray.direction);
	float b = 2 * Vec3Dot(ray.direction, L);
	float c = Vec3Dot(L, L) - sphere.radius_sq;

	if (!SolveQuadratic(a, b, c, t0, t1))
	{
		return false;
	}
#endif
	if (t0 > t1)
	{
		std::swap(t0, t1);
	}

	if (t0 < 0.0f) {
		t0 = t1;

		if (t0 < 0.0f)
		{
			return false;
		}
	}

	if (t0 < ray.t)
	{
		ray.t = t0;
		return true;
	}

	return false;
}

Vec3 GetSphereNormalAtPoint(const Sphere& sphere, const Vec3& point)
{
	return Vec3Normalize(point - sphere.center);
}

Vec3 GetSphereCentroid(const Sphere& sphere)
{
	return sphere.center;
}

AABB GetSphereBounds(const Sphere& sphere)
{
	return { .pmin = sphere.center - std::sqrtf(sphere.radius_sq), .pmax = sphere.center + std::sqrtf(sphere.radius_sq) };
}

bool IntersectTriangle(const Triangle& triangle, Ray& ray)
{
	float t = 0.0f;
#if INTERSECTION_TRIANGLE_INSIDE_OUTSIDE_TEST
	Vec3 v0v1 = triangle.v1 - triangle.v0;
	Vec3 v0v2 = triangle.v2 - triangle.v0;
	Vec3 N = Vec3Cross(v0v1, v0v2);
	float area = Vec3Length(N);

	float NoD = Vec3Dot(N, ray.direction);

	if (std::fabs(NoD) < 0.001f)
	{
		return false;
	}

	float d = Vec3Dot(-N, triangle.v0);
	t = -(Vec3Dot(N, ray.origin) + d);

	if (t < 0.0f)
	{
		return false;
	}

	Vec3 P = ray.origin + t * ray.direction;

	Vec3 edge0 = triangle.v1 - triangle.v0;
	Vec3 vp0 = P - triangle.v0;
	Vec3 C = Vec3Cross(edge0, vp0);

	if (Vec3Dot(N, C) < 0.0f)
	{
		return false;
	}

	Vec3 edge1 = triangle.v2 - triangle.v1;
	Vec3 vp1 = P - triangle.v1;
	C = Vec3Cross(edge1, vp1);

	if (Vec3Dot(N, C) < 0.0f)
	{
		return false;
	}

	Vec3 edge2 = triangle.v0 - triangle.v2;
	Vec3 vp2 = P - triangle.v2;
	C = Vec3Cross(edge2, vp2);

	if (Vec3Dot(N, C) < 0.0f)
	{
		return false;
	}
#elif INTERSECTION_TRIANGLE_MOELLER_TRUMBORE
	Vec3 edge1 = triangle.v1.pos - triangle.v0.pos;
	Vec3 edge2 = triangle.v2.pos - triangle.v0.pos;

	Vec3 H = Vec3Cross(ray.direction, edge2);
	float a = Vec3Dot(edge1, H);

	if (a > -0.001f && a < 0.001f)
	{
		return false;
	}

	float f = 1.0f / a;
	Vec3 S = ray.origin - triangle.v0.pos;
	float u = f * Vec3Dot(S, H);

	if (u < 0.0f || u > 1.0f)
	{
		return false;
	}

	Vec3 Q = Vec3Cross(S, edge1);
	float v = f * Vec3Dot(ray.direction, Q);

	if (v < 0.0f || u + v > 1.0f)
	{
		return false;
	}

	t = f * Vec3Dot(edge2, Q);

	if (t < 0.001f)
	{
		return false;
	}
#endif

	if (t < ray.t)
	{
		ray.t = t;
		return true;
	}

	return false;
}

Vec3 GetTriangleNormalAtPoint(const Triangle& triangle, const Vec3& point)
{
	return triangle.v0.normal;
}

Vec3 GetTriangleCentroid(const Triangle& triangle)
{
	return (triangle.v0.pos + triangle.v1.pos + triangle.v2.pos) * 0.3333f;
}

AABB GetTriangleBounds(const Triangle& triangle)
{
	AABB aabb = { triangle.v0.pos, triangle.v0.pos };

	aabb.pmin = Vec3Min(aabb.pmin, triangle.v1.pos);
	aabb.pmax = Vec3Max(aabb.pmax, triangle.v1.pos);

	aabb.pmin = Vec3Min(aabb.pmin, triangle.v2.pos);
	aabb.pmax = Vec3Max(aabb.pmax, triangle.v2.pos);

	return aabb;
}
