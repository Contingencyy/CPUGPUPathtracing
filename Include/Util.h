#pragma once
#include "MathLib.h"

namespace Util
{

	Vec3 UniformHemisphereSample(const Vec3& normal)
	{
		float r1 = RandomFloat();
		float r2 = RandomFloat();

		float sin_theta = std::sqrtf(1.0f - r1 * r1);
		float phi = 2 * PI * r2;

		float x = sin_theta * cosf(phi);
		float z = sin_theta * sinf(phi);

		Vec3 Nt(0.0f);

		if (std::fabs(normal.x) > std::fabs(normal.y))
		{
			Nt = Vec3(normal.z, 0.0f, -normal.x) / std::sqrtf(normal.x * normal.x + normal.z * normal.z);
		}
		else
		{
			Nt = Vec3(0.0f, -normal.z, normal.y) / std::sqrtf(normal.y * normal.y + normal.z * normal.z);
		}

		Vec3 Nb = Vec3Cross(normal, Nt);
		Vec3 sample(x, r1, z);

		return Vec3Normalize(Vec3(
			sample.x * Nb.x + sample.y * normal.x + sample.z * Nt.x,
			sample.x * Nb.y + sample.y * normal.y + sample.z * Nt.y,
			sample.x * Nb.z + sample.y * normal.z + sample.z * Nt.z
		));
	}

}
