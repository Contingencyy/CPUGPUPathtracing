#include "Common.h"
#include "Primitives.h"

#include "imgui/imgui.h"

bool IntersectTriangle(const Triangle& triangle, Ray& ray)
{
	float t = 0.0f;
	Vec3 edge1 = triangle.v1.pos - triangle.v0.pos;
	Vec3 edge2 = triangle.v2.pos - triangle.v0.pos;

	Vec3 H = Vec3Cross(ray.direction, edge2);
	float a = Vec3Dot(edge1, H);

	// We want to have triangles be double-sided (no back-face culling), since we do dielectrics
	if (std::fabs(a) < 0.001f)
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

	if (t > 0.0f && t < ray.t)
	{
		ray.t = t;
		return true;
	}

	return false;
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

Vec3 TriangleNormal(const Triangle& triangle, const Vec3& pos)
{
	return triangle.v0.normal;
}

Vec3 SphereNormal(const Sphere& sphere, const Vec3& pos)
{
	return Vec3Normalize(pos - sphere.center);
}

Vec3 PlaneNormal(const Plane& plane, const Vec3& pos)
{
	return plane.normal;
}

Vec3 AABBNormal(const AABB& aabb, const Vec3& pos)
{
	EXCEPT("AABBNormal", "Not implemented");
}

// Note: Flipping over the diagonal can reduce usefulness of blue noise or low-discrepancy sampling within the triangle
// There is no guarantee that well distributed points will remain well distributed when folded over the diagonal
Vec3 RandomPointTriangle(const Triangle& triangle)
{
	float u0 = RandomFloat();
	float u1 = RandomFloat();

	float alpha = u0;
	float beta = u1;

	if (alpha + beta > 1.0f)
	{
		alpha = 1.0f - alpha;
		beta = 1.0f - beta;
	}

	float gamma = 1.0f - beta - alpha;
	return alpha * triangle.v0.pos + beta * triangle.v1.pos + gamma * triangle.v2.pos;
}

Vec3 RandomPointSphere(const Sphere& sphere)
{
	Vec3 dir = Vec3(0.0f);
	do
	{
		dir = Vec3(RandomFloat() * 2.0f - 1.0f, RandomFloat() * 2.0f - 1.0f, RandomFloat() * 2.0f - 1.0f);
	} while (Vec3Dot(dir, dir) > 1.0f);

	return sphere.center + sphere.radius * Vec3Normalize(dir);
}

Vec3 RandomPointPlane(const Plane& plane)
{
	EXCEPT("RandomPointPlane", "Not implemented");
}

Vec3 RandomPointAABB(const AABB& aabb)
{
	EXCEPT("RandomPointAABB", "Not implemented");
}

Vec3 RandomPointTriangleFacing(const Triangle& triangle, const Vec3& pos)
{
	EXCEPT("RandomPointTriangleFacing", "Not implemented");
}

Vec3 RandomPointSphereFacing(const Sphere& sphere, const Vec3& pos)
{
	Vec3 to_pos = Vec3Normalize(pos - sphere.center);
	Vec3 dir = Util::UniformHemisphereSample(to_pos);

	return sphere.center + sphere.radius * dir;
}

Vec3 RandomPointPlaneFacing(const Plane& plane, const Vec3& pos)
{
	EXCEPT("RandomPointPlaneFacing", "Not implemented");
}

Vec3 RandomPointAABBFacing(const AABB& aabb, const Vec3& pos)
{
	EXCEPT("RandomPointAABBFacing", "Not implemented");
}

AABB TriangleBounds(const Triangle& triangle)
{
	AABB aabb = { triangle.v0.pos, triangle.v0.pos };

	aabb.pmin = Vec3Min(aabb.pmin, triangle.v1.pos);
	aabb.pmax = Vec3Max(aabb.pmax, triangle.v1.pos);

	aabb.pmin = Vec3Min(aabb.pmin, triangle.v2.pos);
	aabb.pmax = Vec3Max(aabb.pmax, triangle.v2.pos);

	return aabb;
}

AABB SphereBounds(const Sphere& sphere)
{
	return { .pmin = sphere.center - sphere.radius, .pmax = sphere.center + sphere.radius };
}

Vec3 GetSphereCentroid(const Sphere& sphere)
{
	return sphere.center;
}

Vec3 TriangleCentroid(const Triangle& triangle)
{
	return (triangle.v0.pos + triangle.v1.pos + triangle.v2.pos) * 0.3333f;
}

Vec3 SphereCentroid(const Sphere& sphere)
{
	return sphere.center;
}

Vec3 AABBCentroid(const AABB& aabb)
{
	return Vec3Lerp(aabb.pmin, aabb.pmax, 0.5f);
}

float GetTriangleArea(const Triangle& triangle)
{
	float a = Vec3Length(triangle.v1.pos - triangle.v0.pos);
	float b = Vec3Length(triangle.v2.pos - triangle.v0.pos);
	float c = Vec3Length(triangle.v2.pos - triangle.v1.pos);

	float s = (a + b + c) / 2.0f;
	return std::sqrtf(s * (s - a) * (s - b) * (s - c));
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

bool Primitive::Intersect(Ray& ray) const
{
	switch (type)
	{
	case PrimitiveType_Triangle:
		return IntersectTriangle(triangle, ray);
	case PrimitiveType_Sphere:
		return IntersectSphere(sphere, ray);
	case PrimitiveType_Plane:
		return IntersectPlane(plane, ray);
	case PrimitiveType_AABB:
	default:
		EXCEPT("Primitive", "Intersection not implemented for this primitive type");
		break;
	}
}

Vec3 Primitive::Normal(const Vec3& pos) const
{
	switch (type)
	{
	case PrimitiveType_Triangle:
		return TriangleNormal(triangle, pos);
	case PrimitiveType_Sphere:
		return SphereNormal(sphere, pos);
	case PrimitiveType_Plane:
		return PlaneNormal(plane, pos);
	case PrimitiveType_AABB:
		return AABBNormal(aabb, pos);
	}
}

Vec3 Primitive::RandomPoint() const
{
	switch (type)
	{
	case PrimitiveType_Triangle:
		return RandomPointTriangle(triangle);
	case PrimitiveType_Sphere:
		return RandomPointSphere(sphere);
	case PrimitiveType_Plane:
		return RandomPointPlane(plane);
	case PrimitiveType_AABB:
		return RandomPointAABB(aabb);
	}
}

Vec3 Primitive::RandomPointFacing(const Vec3& pos) const
{
	switch (type)
	{
	case PrimitiveType_Triangle:
		return RandomPointTriangleFacing(triangle, pos);
	case PrimitiveType_Sphere:
		return RandomPointSphereFacing(sphere, pos);
	case PrimitiveType_Plane:
		return RandomPointPlaneFacing(plane, pos);
	case PrimitiveType_AABB:
		return RandomPointAABBFacing(aabb, pos);
	}
}

AABB Primitive::Bounds() const
{
	switch (type)
	{
	case PrimitiveType_Triangle:
		return TriangleBounds(triangle);
	case PrimitiveType_Sphere:
		return SphereBounds(sphere);
	case PrimitiveType_Plane:
	case PrimitiveType_AABB:
		EXCEPT("Primitive::Bounds", "Called on PrimitiveType_Plane or PrimitiveType_AABB, which is invalid on these types");
		break;
	}
}

Vec3 Primitive::Centroid() const
{
	switch (type)
	{
	case PrimitiveType_Triangle:
		return TriangleCentroid(triangle);
	case PrimitiveType_Sphere:
		return SphereCentroid(sphere);
	case PrimitiveType_AABB:
		return AABBCentroid(aabb);
	case PrimitiveType_Plane:
		EXCEPT("Primitive::Centroid", "Called on PrimitiveType_Plane, which is invalid for that type");
		break;
	}
}

void Primitive::RenderImGui()
{
	switch (type)
	{
		case PrimitiveType_Triangle:
		{
		} break;
		case PrimitiveType_Sphere:
		{
			Sphere& sphere = (Sphere&)*this;

			ImGui::Text("Sphere");
			ImGui::SliderFloat3("Center", &sphere.center.x, -FLT_MAX, FLT_MAX, "%.3f");
			ImGui::SliderFloat("Radius squared", &sphere.radius_sq, -10000.0f, 10000.0f, "%.3f");
		} break;
		case PrimitiveType_Plane:
		{
			Plane& plane = (Plane&)*this;
			
			ImGui::Text("Plane");
			ImGui::SliderFloat3("Point", &plane.point.x, -FLT_MAX, FLT_MAX, "%.3f");
			if (ImGui::SliderFloat3("Normal", &plane.normal.x, 0.0f, 1.0f, "%.3f"))
			{
				plane.normal = Vec3Normalize(plane.normal);
			}
		} break;
		case PrimitiveType_AABB:
		{
		} break;
	}
}
