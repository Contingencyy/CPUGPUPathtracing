#include "Common.h"
#include "Util.h"

namespace Util
{

	Vec3 UniformHemisphereSample(const Vec3& normal)
	{
		Vec3 dir = Vec3(0.0f);
		do
		{
			dir = Vec3(RandomFloat() * 2.0f - 1.0f, RandomFloat() * 2.0f - 1.0f, RandomFloat() * 2.0f - 1.0f);
		} while (Vec3Dot(dir, dir) > 1.0f);

		if (Vec3Dot(dir, normal) < 0.0f)
			dir *= -1.0f;

		return Vec3Normalize(dir);
	}

	Vec3 CosineWeightedDiffuseReflection(const Vec3& normal)
	{
		Vec3 dir = Vec3(0.0f);
		do
		{
			dir = Vec3(RandomFloat() * 2.0f - 1.0f, RandomFloat() * 2.0f - 1.0f, RandomFloat() * 2.0f - 1.0f);
		} while (Vec3Dot(dir, dir) > 1.0f);

		return Vec3Normalize(normal + Vec3Normalize(dir));
	}

	float SurvivalProbabilityRR(const Vec3& albedo)
	{
		return std::clamp(std::max(std::max(albedo.x, albedo.y), albedo.z), 0.1f, 1.0f);
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

	Vec3 LessThan(const Vec3& v0, float value)
	{
		return Vec3(v0.x < value ? 1.0f : 0.0f, v0.y < value ? 1.0f : 0.0f, v0.z < value ? 1.0f : 0.0f);
	}

	Vec3 LinearToSRGB(const Vec3& rgb)
	{
		Vec3 clamped = Vec3Clamp(rgb, 0.0f, 1.0f);
		return Vec3Lerp(
			Vec3Pow(clamped, (1.0f / 2.4f) * 1.055f - 0.055f),
			clamped * 12.92f,
			LessThan(clamped, 0.0031308f)
		);
	}

	Vec3 SRGBToLinear(const Vec3& rgb)
	{
		Vec3 clamped = Vec3Clamp(rgb, 0.0f, 1.0f);
		return Vec3Lerp(
			Vec3Pow((clamped + 0.055f) / 1.055f, 2.4f),
			clamped / 12.92f,
			LessThan(clamped, 0.04045f)
		);
	}

}
