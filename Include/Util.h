#pragma once

namespace Util
{

	Vec3 UniformHemisphereSample(const Vec3& normal);
	Vec3 CosineWeightedDiffuseReflection(const Vec3& normal);
	float SurvivalProbabilityRR(const Vec3& albedo);

	Vec3 Reflect(const Vec3& dir, const Vec3& normal);
	float Fresnel(float in, float out, float ior_outside, float ior_inside);
	Vec3 Refract(const Vec3& dir, const Vec3& normal, float eta, float cosi, float k);

	Vec3 LessThan(const Vec3& v0, float value);
	Vec3 LinearToSRGB(const Vec3& rgb);
	Vec3 SRGBToLinear(const Vec3& rgb);

}
