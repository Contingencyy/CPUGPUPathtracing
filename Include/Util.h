#pragma once
#include "MathLib.h"

namespace Util
{

	Vec3 UniformHemisphereSample(const Vec3& normal)
	{
		Vec3 dir = Vec3(RandomFloat() * 2 - 1, RandomFloat() * 2 - 1, RandomFloat() * 2 - 1);

		if (Vec3Dot(dir, normal) < 0.0f)
			dir *= -1.0f;

		return Vec3Normalize(dir);
	}

	Vec3 Reflect(const Vec3& dir, const Vec3& normal)
	{
		return dir - 2.0f * normal * Vec3Dot(dir, normal);
	}

	float Fresnel(float in, float out, float ior_outside, float ior_inside)
	{
		float sPolarized = (ior_outside * in - ior_inside * out) /
			(ior_outside * in + ior_inside * out);
		float pPolarized = (ior_outside * out - ior_inside * in) /
			(ior_outside * out + ior_inside * in);
		return 0.5f * ((sPolarized * sPolarized) + (pPolarized * pPolarized));
	}

	Vec3 Refract(const Vec3& dir, const Vec3& normal, float eta, float cosi, float k)
	{
		return Vec3Normalize(dir * eta + ((eta * cosi - std::sqrtf(k)) * normal));
	}

}
