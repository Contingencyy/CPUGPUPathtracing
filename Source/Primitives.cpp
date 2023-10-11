#include "Primitives.h"

bool IntersectAABB(const AABB& aabb, Ray& ray)
{
	float tx1 = (aabb.pmin.x - ray.origin.x) / ray.direction.x, tx2 = (aabb.pmax.x - ray.origin.x) / ray.direction.x;
	float tmin = std::min(tx1, tx2), tmax = std::max(tx1, tx2);
	float ty1 = (aabb.pmin.y - ray.origin.y) / ray.direction.y, ty2 = (aabb.pmax.y - ray.origin.y) / ray.direction.y;
	tmin = std::max(tmin, std::min(ty1, ty2)), tmax = std::min(tmax, std::max(ty1, ty2));
	float tz1 = (aabb.pmin.z - ray.origin.z) / ray.direction.z, tz2 = (aabb.pmax.z - ray.origin.z) / ray.direction.z;
	tmin = std::max(tmin, std::min(tz1, tz2)), tmax = std::min(tmax, std::max(tz1, tz2));
	return tmax >= tmin && tmin < ray.t && tmax > 0.0f;
}

void IntersectPlane(const Plane& plane, Ray& ray)
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
			ray.payload.object_type = PrimitiveType_Plane;
			ray.payload.object_ptr = (void*)&plane;
			ray.payload.mat_index = plane.mat_index;
		}
	}
}

void IntersectSphere(const Sphere& sphere, Ray& ray)
{
	float t0, t1;
#if INTERSECTION_SPHERE_GEOMETRIC
	// Seems to be faster than solving the quadratic
	Vec3 L = sphere.center - ray.origin;
	float tca = Vec3Dot(L, ray.direction);

	if (tca < 0.0f)
	{
		return;
	}

	float d2 = Vec3Dot(L, L) - tca * tca;

	if (d2 > sphere.radius_sq)
	{
		return;
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
		return;
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
			return;
		}
	}

	if (t0 < ray.t)
	{
		ray.t = t0;
		ray.payload.object_type = PrimitiveType_Sphere;
		ray.payload.object_ptr = (void*)&sphere;
		ray.payload.mat_index = sphere.mat_index;
	}
}

static Vec3 GetTriangleNormal(const Triangle& triangle)
{
	Vec3 v0v1 = triangle.v1 - triangle.v0;
	Vec3 v0v2 = triangle.v2 - triangle.v0;
	return Vec3Normalize(Vec3Cross(v0v1, v0v2));
}

void IntersectTriangle(const Triangle& triangle, Ray& ray)
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
		return;
	}

	float d = Vec3Dot(-N, triangle.v0);
	t = -(Vec3Dot(N, ray.origin) + d);

	if (t < 0.0f)
	{
		return;
	}

	Vec3 P = ray.origin + t * ray.direction;

	Vec3 edge0 = triangle.v1 - triangle.v0;
	Vec3 vp0 = P - triangle.v0;
	Vec3 C = Vec3Cross(edge0, vp0);

	if (Vec3Dot(N, C) < 0.0f)
	{
		return;
	}

	Vec3 edge1 = triangle.v2 - triangle.v1;
	Vec3 vp1 = P - triangle.v1;
	C = Vec3Cross(edge1, vp1);

	if (Vec3Dot(N, C) < 0.0f)
	{
		return;
	}

	Vec3 edge2 = triangle.v0 - triangle.v2;
	Vec3 vp2 = P - triangle.v2;
	C = Vec3Cross(edge2, vp2);

	if (Vec3Dot(N, C) < 0.0f)
	{
		return;
	}
#elif INTERSECTION_TRIANGLE_MOELLER_TRUMBORE
	Vec3 edge1 = triangle.v1 - triangle.v0;
	Vec3 edge2 = triangle.v2 - triangle.v0;

	Vec3 H = Vec3Cross(ray.direction, edge2);
	float a = Vec3Dot(edge1, H);

	if (a > -0.001f && a < 0.001f)
	{
		return;
	}

	float f = 1.0f / a;
	Vec3 S = ray.origin - triangle.v0;
	float u = f * Vec3Dot(S, H);

	if (u < 0.0f || u > 1.0f)
	{
		return;
	}

	Vec3 Q = Vec3Cross(S, edge1);
	float v = f * Vec3Dot(ray.direction, Q);

	if (v < 0.0f || u + v > 1.0f)
	{
		return;
	}

	t = f * Vec3Dot(edge2, Q);

	if (t < 0.001f)
	{
		return;
	}
#endif

	if (t < ray.t)
	{
		ray.t = t;
		ray.payload.object_type = PrimitiveType_Triangle;
		ray.payload.object_ptr = (void*)&triangle;
		ray.payload.mat_index = triangle.mat_index;
	}
}

Vec3 GetObjectSurfaceNormalAtPoint(PrimitiveType type, void* ptr, const Vec3& point)
{
	switch (type)
	{
	case PrimitiveType_Plane:
	{
		Plane* plane = reinterpret_cast<Plane*>(ptr);
		return plane->normal;
	} break;
	case PrimitiveType_Sphere:
	{
		Sphere* sphere = reinterpret_cast<Sphere*>(ptr);
		return Vec3Normalize(point - sphere->center);
	} break;
	case PrimitiveType_Triangle:
	{
		Triangle* triangle = reinterpret_cast<Triangle*>(ptr);
		return GetTriangleNormal(*triangle);
	} break;
	}
}
