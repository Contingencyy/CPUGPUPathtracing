#include "Common.h"
#include "DX12.h"
#include "Window.h"
#include "Input.h"
#include "ThreadPool.h"
#include "Primitives.h"
#include "BVH.h"
#include "GLTFLoader.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#ifdef CreateWindow
#undef CreateWindow
#endif

#ifdef LoadImage
#undef LoadImage
#endif

#ifdef OPAQUE
#undef OPAQUE
#endif

#ifdef TRANSPARENT
#undef TRANSPARENT
#endif

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

#ifdef near
#undef near
#endif

#ifdef far
#undef far
#endif

#include "imgui/imgui.h"
#include "imgui/imgui_impl_win32.h"
#include "imgui/imgui_impl_dx12.h"

static constexpr float RAY_REFLECT_NUDGE_MULTIPLIER = 0.001f;

struct Material
{
	Vec3 albedo = Vec3(0.0f);
	float specular = 0.0f;

	float refractivity = 0.0f;
	Vec3 absorption = Vec3(0.0f);
	float ior = 1.0f;

	Vec3 emissive = Vec3(0.0f);
	float intensity = 0.0f;
	bool is_light = false;

	Material(const Vec3& albedo, float spec)
		: albedo(albedo), specular(spec) {}
	Material(const Vec3& albedo, float spec, float refract, const Vec3& absorption, float ior)
		: albedo(albedo), specular(spec), refractivity(refract), absorption(absorption), ior(ior) {}
	Material(const Vec3& emissive, float intensity, bool light)
		: emissive(emissive), intensity(intensity), is_light(light) {}

	bool RenderImGui()
	{
		bool changed = false;

		changed = ImGui::ColorEdit3("Albedo", &albedo.x, ImGuiColorEditFlags_DisplayRGB);
		changed = ImGui::SliderFloat("Specular", &specular, 0.0f, 1.0f, "%.3f");

		ImGui::Separator();

		changed = ImGui::SliderFloat("Refractivity", &refractivity, 0.0f, 1.0f, "%.3f");
		changed = ImGui::ColorEdit3("Absorption", &absorption.x, ImGuiColorEditFlags_DisplayRGB);
		changed = ImGui::SliderFloat("IOR", &ior, 0.0f, 10.0f, "%.3f");

		ImGui::Separator();

		changed = ImGui::ColorEdit3("Emissive", &emissive.x, ImGuiColorEditFlags_DisplayRGB);
		changed = ImGui::SliderFloat("Intensity", &intensity, 0.0f, 1000.0f, "%.3f");
		changed = ImGui::Checkbox("Is light", &is_light);

		return changed;
	}
};

class Camera
{
public:
	Camera() = default;
	Camera(const Vec3& pos, const Vec3& view_dir, float fov, float aspect)
		: m_pos(pos), m_view_dir(view_dir), m_fov(Deg2Rad(fov)), m_aspect(aspect)
	{
		UpdateScreenPlane();
	}

	bool Update(float dt)
	{
		bool view_changed = false;

		// Camera look
		Vec2 mouse_move = Input::GetMouseMoveRel();

		// Camera movement
		float right_velocity = Input::GetInputAxis1D(Input::KeyCode_A, Input::KeyCode_D) * dt * m_speed;
		float up_velocity = Input::GetInputAxis1D(Input::KeyCode_Space, Input::KeyCode_LeftShift) * dt * m_speed;
		float forward_velocity = Input::GetInputAxis1D(Input::KeyCode_W, Input::KeyCode_S) * dt * m_speed;

		m_pos.x -= right_velocity;
		m_pos.y += up_velocity;
		m_pos.z -= forward_velocity;

		if (right_velocity != 0.0f || up_velocity != 0.0f || forward_velocity != 0.0f)
		{
			view_changed = true;
		}

		if (view_changed)
		{
			UpdateScreenPlane();
		}

		return view_changed;
	}

	Ray GetRay(float u, float v)
	{
		Vec3 pixel_pos = m_screen_plane.top_left +
			u * (m_screen_plane.top_right - m_screen_plane.top_left) +
			v * (m_screen_plane.bottom_left - m_screen_plane.top_left);

		return Ray(m_pos, Vec3Normalize(pixel_pos - m_pos));
	}

private:
	void UpdateScreenPlane()
	{
		m_screen_plane.center = m_pos + m_fov * m_view_dir;
		m_screen_plane.top_left = m_screen_plane.center + Vec3(-m_aspect, 1.0f, 0.0f);
		m_screen_plane.top_right = m_screen_plane.center + Vec3(m_aspect, 1.0f, 0.0f);
		m_screen_plane.bottom_left = m_screen_plane.center + Vec3(-m_aspect, -1.0f, 0.0f);
	}

private:
	Vec3 m_pos = Vec3(0.0f);
	Vec3 m_view_dir = Vec3(0.0f, 0.0f, -1.0f);

	float m_fov = Deg2Rad(60.0f);
	float m_aspect = 16.0f / 9.0f;

	float m_pitch = 0.0f;
	float m_yaw = 0.0f;
	float m_speed = 2.0f;

	struct ScreenPlane
	{
		Vec3 center = Vec3(0.0f);
		Vec3 top_left = Vec3(0.0f);
		Vec3 top_right = Vec3(0.0f);
		Vec3 bottom_left = Vec3(0.0f);
	} m_screen_plane;

};

enum RenderMode
{
	RENDER_MODE_COMPARISON,
	RENDER_MODE_BRUTE_FORCE,
	RENDER_MODE_NEXT_EVENT_EST,
	RENDER_MODE_NUM_MODES
};

const std::vector<const char*> render_mode_labels =
{
	"Comparison", "Brute force", "Next event estimation"
};

enum DebugRenderMode
{
	DEBUG_RENDER_MODE_NONE,
	DEBUG_RENDER_MODE_NUM_MODES
};

const std::vector<const char*> debug_render_mode_labels =
{
	"None"
};

struct Object;

struct Data
{
	// Render data
	std::vector<uint32_t> pixels;
	std::vector<Vec4> accumulator;
	uint32_t num_accumulated = 0;
	double total_energy_received = 0.0f;
	bool pause_rendering = false;
	int32_t max_ray_depth = 5;

	std::vector<Object> objects;
	std::vector<uint32_t> light_source_indices;

	std::vector<Material> materials;
	Camera camera;

	RenderMode render_mode = RENDER_MODE_COMPARISON;
	DebugRenderMode debug_render_mode = DEBUG_RENDER_MODE_NONE;

	struct Statistics
	{
		uint32_t traced_rays = 0;

		void Reset()
		{
			traced_rays = 0;
		}
	} stats;
} static data;

void ResetAccumulator()
{
	data.num_accumulated = 0;
	std::fill(data.accumulator.begin(), data.accumulator.end(), Vec4(0.0f));
	data.total_energy_received = 0.0f;
}

struct Object
{
	Object(const char* name, const Mesh& mesh, uint32_t mat_index, BVH::BuildOption build_option)
		: name(name), mat_index(mat_index), has_bvh(true)
	{
		bvh.Build(mesh.vertices, mesh.indices, build_option);
	}

	Object(const char* name, const Primitive& prim, uint32_t mat_index)
		: name(name), mat_index(mat_index), primitive(prim) {}

	void RenderImGui()
	{
		if (has_bvh)
			bvh.RenderImGui();
		else
			primitive.RenderImGui();

		bool material_changed = data.materials[mat_index].RenderImGui();
		if (material_changed)
			ResetAccumulator();
	}

	uint32_t mat_index = 0;
	const char* name = "";

	bool has_bvh = false;
	BVH bvh;

	Primitive primitive;
};

void Update(float dt)
{
	Input::UpdateMousePosition();
	if (!ImGui::GetIO().WantCaptureMouse)
	{
		if (Input::IsKeyPressed(Input::KeyCode_LeftMouse))
		{
			Window::SetMouseCapture(true);
		}
	}
	if (Input::IsKeyPressed(Input::KeyCode_RightMouse))
	{
		Window::SetMouseCapture(false);
	}

	bool view_changed = data.camera.Update(dt);
	if (view_changed)
	{
		ResetAccumulator();
	}
}

void IntersectScene(Ray& ray)
{
	data.stats.traced_rays++;

	for (uint32_t obj_idx = 0; obj_idx < data.objects.size(); ++obj_idx)
	{
		const Object& obj = data.objects[obj_idx];
		bool hit = false;

		if (obj.has_bvh)
			hit = obj.bvh.Traverse(ray);
		else
			hit = obj.primitive.Intersect(ray);

		if (hit)
			ray.payload.obj_idx = obj_idx;
	}
}

struct HitResult
{
	Vec3 pos = Vec3(0.0f);
	Vec3 normal = Vec3(0.0f);
	Material* mat = nullptr;
};

HitResult GetRayHitResult(const Ray& ray)
{
	HitResult result = {};
	result.pos = ray.origin + ray.direction * ray.t;
	const Object& hit_obj = data.objects[ray.payload.obj_idx];

	if (hit_obj.has_bvh)
		result.normal = TriangleNormal(hit_obj.bvh.GetTriangle(ray.payload.tri_idx), result.pos);
	else
		result.normal = hit_obj.primitive.Normal(result.pos);

	result.mat = &data.materials[data.objects[ray.payload.obj_idx].mat_index];
	return result;
}

struct LightSample
{
	Vec3 pos = Vec3(0.0f);
	Vec3 normal = Vec3(0.0f, -1.0f, 0.0f);
	Vec3 emission = Vec3(0.0f);
	float solid_angle = 0.0f;
	uint32_t obj_idx = ~0u;
};

LightSample GetRandomLightSourceForSample(const Vec3& hit_pos)
{
	LightSample sample = {};

	if (data.light_source_indices.size() > 0)
	{
		sample.obj_idx = data.light_source_indices[RandomUInt32Range(0u, (uint32_t)data.light_source_indices.size() - 1)];
		const Object& light_source = data.objects[sample.obj_idx];

 		if (light_source.has_bvh)
		{
			EXCEPT("GetRandomLightSourceForSample", "Tried to get a sample from a light source that has a BVH, which is currently not implemented");
		}
		else
		{
			if (light_source.primitive.type == PrimitiveType_Sphere)
			{
				sample.pos = light_source.primitive.RandomPointFacing(hit_pos);
				//sample.pos = light_source.primitive.RandomPoint();
				sample.normal = light_source.primitive.Normal(sample.pos);
				sample.emission = data.materials[light_source.mat_index].emissive * data.materials[light_source.mat_index].intensity;

				float dist_to_light = Vec3Length(sample.pos - hit_pos);
				sample.solid_angle = (2.0f * PI * light_source.primitive.sphere.radius_sq) / (dist_to_light * dist_to_light);
				//sample.solid_angle = (4.0f * PI * light_source.primitive.sphere.radius_sq) / (dist_to_light * dist_to_light);
			}
			else
			{
				EXCEPT("GetRandomLightSourceForSample", "Tried to get a sample from a light source with a primitive type other than PrimitiveType_Sphere");
			}
		}
	}

	return sample;
}

//Vec4 TracePath(Ray& ray)
//{
//	Vec3 throughput = Vec3(1.0f);
//	Vec3 energy = Vec3(0.0f);
//	Vec3 debug_color = Vec3(0.0f);
//
//	uint8_t ray_depth = 0;
//	bool last_specular = false;
//
//	//while (true)
//	while (ray_depth <= data.max_ray_depth)
//	{
//		IntersectScene(ray);
//
//		if (ray_depth == 0 && data.debug_view == DebugRenderView_BVHDepth)
//			debug_color = Vec3Lerp(Vec3(0.0f, 1.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), std::min(1.0f, (float)ray.payload.bvh_depth / 30.0f));
//
//		// We have not hit any geometry, so we stop the path
//		if (ray.payload.obj_idx == ~0u)
//		{
//			/*if (ray_depth == 0)
//			{
//				energy = Vec3(0.55f, 0.8f, 0.9f);
//			}*/
//			break;
//		}
//
//		HitResult hit_result = GetRayHitResult(ray);
//
//		// We have hit a light source, so we stop
//		if (hit_result.mat->is_light)
//		{
//			if (ray_depth == 0 || last_specular)
//			{
//				energy += throughput * hit_result.mat->emissive * hit_result.mat->intensity;
//				last_specular = false;
//			}
//			break;
//		}
//
//		Vec3 brdf_diffuse = hit_result.mat->albedo * INV_PI;
//
//		// Next event estimation, evaluate both direct and indirect light at vertex
//		// Direct lighting - Sample a random light source
//		float diffuse_weight = std::max(0.0f, 1.0f - (hit_result.mat->specular + hit_result.mat->refractivity));
//		if (data.light_source_indices.size() > 0 && diffuse_weight > 0.0f)
//		{
//			// TODO: The skybox should also be a light source, currently its not accounted for
//			Vec3 light_pos = Vec3(0.0f);
//			Vec3 light_normal = Vec3(0.0f, -1.0f, 0.0f);
//			Vec3 light_emission = Vec3(0.0f);
//			float light_area = 0.0f;
//			uint32_t light_obj_idx = ~0u;
//			GetRandomLightSourceForSample(light_pos, light_normal, light_emission, light_area, light_obj_idx);
//
//			Vec3 vert_to_light = light_pos - hit_result.pos;
//			float dist_to_light = Vec3Length(vert_to_light);
//			vert_to_light = Vec3Normalize(vert_to_light);
//
//			float NdotL = Vec3Dot(hit_result.normal, vert_to_light);
//			float NLdotL = Vec3Dot(light_normal, -vert_to_light);
//
//			// We dont need to trace a ray to the random point on the light if they dont face each other
//			if (NdotL > 0.0f && NLdotL > 0.0f)
//			{
//				Ray shadow_ray(hit_result.pos + vert_to_light * RAY_REFLECT_NUDGE_MULTIPLIER, vert_to_light, dist_to_light);
//				IntersectScene(shadow_ray);
//				bool occluded = shadow_ray.payload.obj_idx != light_obj_idx;
//
//				// If the ray towards the light source was not occluded, we need to determine the energy coming from the light source
//				if (!occluded)
//				{
//					float solid_angle = (NLdotL * light_area) / (dist_to_light * dist_to_light);
//					float light_pdf = 1.0f / solid_angle;
//
//					// TODO: Is this correct? Do we need to adjust the weight of the direct light like this?
//					energy += light_emission * solid_angle * brdf_diffuse * NdotL;
//					//energy += throughput * (NdotL / light_pdf) * brdf_diffuse * light_emission * diffuse_weight;
//				}
//			}
//		}
//
//		// Indirect lighting - Choose a path between diffuse, specular, or refracted
//		{
//			float r = RandomFloat();
//
//			// Specular reflection
//			if (r < hit_result.mat->specular)
//			{
//				Vec3 specular_dir = Util::Reflect(ray.direction, hit_result.normal);
//				ray = Ray(hit_result.pos + specular_dir * RAY_REFLECT_NUDGE_MULTIPLIER, specular_dir);
//				throughput *= hit_result.mat->albedo;
//				last_specular = true;
//			}
//			// Dielectrics
//			else if (r < hit_result.mat->specular + hit_result.mat->refractivity)
//			{
//				Vec3 N = hit_result.normal;
//
//				float cosi = std::clamp(Vec3Dot(N, ray.direction), -1.0f, 1.0f);
//				float etai = 1.0f, etat = hit_result.mat->ior;
//
//				float Fr = 1.0f;
//				bool inside = true;
//
//				if (cosi < 0.0f)
//				{
//					cosi = -cosi;
//					inside = false;
//				}
//				else
//				{
//					std::swap(etai, etat);
//					N = -N;
//				}
//
//				float eta = etai / etat;
//				float k = 1.0f - eta * eta * (1.0f - cosi * cosi);
//
//				if (k >= 0.0f)
//				{
//					Vec3 refract_dir = Util::Refract(ray.direction, N, eta, cosi, k);
//
//					float angle_in = Vec3Dot(ray.direction, hit_result.normal);
//					float angle_out = Vec3Dot(refract_dir, hit_result.normal);
//
//					Fr = Util::Fresnel(angle_in, angle_out, etai, etat);
//					if (RandomFloat() > Fr)
//					{
//						// TODO: Fix this, it no longer is applied to the correct ray
//						if (inside)
//						{
//							Vec3 absorption(0.0f);
//							absorption.x = std::expf(-hit_result.mat->absorption.x * ray.t);
//							absorption.y = std::expf(-hit_result.mat->absorption.y * ray.t);
//							absorption.z = std::expf(-hit_result.mat->absorption.z * ray.t);
//
//							throughput *= absorption;
//						}
//
//						ray = Ray(hit_result.pos + refract_dir * RAY_REFLECT_NUDGE_MULTIPLIER, refract_dir);
//						throughput *= hit_result.mat->albedo;
//					}
//					else
//					{
//						Vec3 specular_dir = Util::Reflect(ray.direction, hit_result.normal);
//						ray = Ray(hit_result.pos + specular_dir * RAY_REFLECT_NUDGE_MULTIPLIER, specular_dir);
//						throughput *= hit_result.mat->albedo;
//						last_specular = true;
//					}
//				}
//			}
//			// Diffuse
//			else
//			{
//				// TODO: Cosine weighted diffuse reflection
//				Vec3 diffuse_dir = Util::UniformHemisphereSample(hit_result.normal);
//				float NdotR = Vec3Dot(hit_result.normal, diffuse_dir);
//				float hemisphere_pdf = 1.0f / (2.0f * PI);
//
//				ray = Ray(hit_result.pos + diffuse_dir * RAY_REFLECT_NUDGE_MULTIPLIER, diffuse_dir);
//				throughput *= (NdotR / hemisphere_pdf) * brdf_diffuse;
//			}
//		}
//
//		if (ray_depth == 0 && data.debug_view == DebugRenderView_Throughput)
//			debug_color = throughput;
//		if (ray_depth == 0 && data.debug_view == DebugRenderView_Normal)
//			debug_color = hit_result.normal;
//
//		ray_depth++;
//	}
//
//	if (data.debug_view == DebugRenderView_RayDepth)
//		debug_color = Vec3Lerp(Vec3(0.0f, 1.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), std::min(1.0f, ((float)ray_depth + 1.0f) / ((float)data.max_ray_depth + 1.0f)));
//
//	if (data.debug_view != DebugRenderView_None)
//		return Vec4(debug_color, 1.0f);
//
//	return Vec4(energy, 1.0f);
//}

Vec4 TracePathNEE(Ray& ray, uint8_t ray_depth, bool last_specular)
{
	// --------------------------------------------------------------------------------------
	// Scene traversal, checking for special cases, etc.

	Vec4 final_color = Vec4(0.0f, 0.0f, 0.0f, 1.0f);

	// We have exceeded the maximum amount of ray recursion, terminate path
	if (ray_depth > data.max_ray_depth)
		return final_color;

	IntersectScene(ray);

	// We have not hit any geometry, sample sky or return black
	if (ray.payload.obj_idx == ~0u)
		return final_color;

	HitResult hit = GetRayHitResult(ray);

	// If we hit a light source, we want to return black, unless we are currently tracing a specular ray
	if (hit.mat->is_light)
	{
		if (ray_depth == 0 || last_specular)
		{
			return Vec4(hit.mat->emissive * hit.mat->intensity, 1.0f);
		}
		return final_color;
	}

	Vec3 brdf_diffuse = hit.mat->albedo * INV_PI;
	Vec3 direct_light = Vec3(0.0f);
	Vec3 indirect_light = Vec3(0.0f);

	// --------------------------------------------------------------------------------------
	// Next event estimation - Evaluate direct and indirect diffuse light at vertex along the path

	if (data.light_source_indices.size() > 0)
	{
		// TODO: The skybox should also be a light source, currently its not accounted for
		LightSample light_sample = GetRandomLightSourceForSample(hit.pos);
		
		Vec3 vert_to_light = light_sample.pos - hit.pos;
		float dist_to_light = Vec3Length(vert_to_light);
		vert_to_light = Vec3Normalize(vert_to_light);
		
		float NdotL = Vec3Dot(hit.normal, vert_to_light);
		float NLdotL = Vec3Dot(light_sample.normal, -vert_to_light);
		
		// We dont need to trace a ray to the random point on the light if they dont face each other
		if (NdotL > 0.0f && NLdotL > 0.0f)
		{
			Ray shadow_ray(hit.pos + vert_to_light * RAY_REFLECT_NUDGE_MULTIPLIER, vert_to_light, dist_to_light - 2.0f * RAY_REFLECT_NUDGE_MULTIPLIER);
			IntersectScene(shadow_ray);
			bool occluded = shadow_ray.payload.obj_idx != ~0u;
		
			// If the ray towards the light source was not occluded, we need to determine the energy coming from the light source
			if (!occluded)
			{
				// This should happen in the diffuse light sampling, so that the chance of going this path
				// is already proportionate to the specularity/refractivity
				// Or we adjust the weight here based on (1 - specularity - reflectivity)
				float solid_angle = NLdotL * light_sample.solid_angle;
				direct_light = light_sample.emission * solid_angle * brdf_diffuse * NdotL * data.light_source_indices.size();
			}
		}
	}

	// Indirect light
	{
		Vec3 diffuse_dir = Util::UniformHemisphereSample(hit.normal);
		Ray diffuse_ray(hit.pos + diffuse_dir * RAY_REFLECT_NUDGE_MULTIPLIER, diffuse_dir);
		float NdotR = Vec3Dot(diffuse_dir, hit.normal);
		Vec4 irradiance = NdotR * TracePathNEE(diffuse_ray, ray_depth + 1, false);

		indirect_light = 2.0f * PI * brdf_diffuse * irradiance.xyz;
	}

	final_color.xyz = direct_light + indirect_light;
	return final_color;
}

Vec4 TracePath(Ray& ray, uint8_t ray_depth)
{
	// --------------------------------------------------------------------------------------
	// Scene traversal, checking for special cases, etc.

	Vec4 final_color = Vec4(0.0f, 0.0f, 0.0f, 1.0f);

	// We have exceeded the maximum amount of ray recursion, terminate path
	if (ray_depth > data.max_ray_depth)
		return final_color;

	IntersectScene(ray);

	// We have not hit any geometry, sample sky or return black
	if (ray.payload.obj_idx == ~0u)
		return final_color;

	HitResult hit = GetRayHitResult(ray);

	// If we hit a light source, we want to return black, unless we are currently tracing a specular ray
	if (hit.mat->is_light)
	{
		return Vec4(hit.mat->emissive * hit.mat->intensity, 1.0f);
	}

	float r = RandomFloat();

	// Specular reflection
	if (r < hit.mat->specular)
	{
		Vec3 specular_dir = Util::Reflect(ray.direction, hit.normal);
		Ray specular_ray(hit.pos + specular_dir * RAY_REFLECT_NUDGE_MULTIPLIER, specular_dir);
		final_color.xyz += hit.mat->albedo * TracePath(specular_ray, ray_depth + 1).xyz;
	}
	// Dielectrics
	else if (r < hit.mat->specular + hit.mat->refractivity)
	{
		Vec3 N = hit.normal;

		float cosi = std::clamp(Vec3Dot(N, ray.direction), -1.0f, 1.0f);
		float etai = 1.0f, etat = hit.mat->ior;

		float Fr = 1.0f;
		bool inside = true;

		if (cosi < 0.0f)
		{
			cosi = -cosi;
			inside = false;
		}
		else
		{
			std::swap(etai, etat);
			N = -N;
		}

		float eta = etai / etat;
		float k = 1.0f - eta * eta * (1.0f - cosi * cosi);

		if (k >= 0.0f)
		{
			Vec3 refract_dir = Util::Refract(ray.direction, N, eta, cosi, k);
			Ray refract_ray(hit.pos + refract_dir * RAY_REFLECT_NUDGE_MULTIPLIER, refract_dir);

			float angle_in = Vec3Dot(ray.direction, hit.normal);
			float angle_out = Vec3Dot(refract_dir, hit.normal);

			Fr = Util::Fresnel(angle_in, angle_out, etai, etat);
			if (RandomFloat() > Fr)
			{
				final_color.xyz += hit.mat->albedo * TracePath(refract_ray, ray_depth + 1).xyz;

				if (inside)
				{
					Vec3 absorption(0.0f);
					absorption.x = std::expf(-hit.mat->absorption.x * ray.t);
					absorption.y = std::expf(-hit.mat->absorption.y * ray.t);
					absorption.z = std::expf(-hit.mat->absorption.z * ray.t);

					final_color.xyz *= absorption;
				}
			}
			else
			{
				Vec3 specular_dir = Util::Reflect(ray.direction, hit.normal);
				Ray specular_ray(hit.pos + specular_dir * RAY_REFLECT_NUDGE_MULTIPLIER, specular_dir);
				final_color.xyz += hit.mat->albedo * TracePath(specular_ray, ray_depth + 1).xyz;
			}
		}
	}
	// Diffuse
	{
		Vec3 diffuse_dir = Util::UniformHemisphereSample(hit.normal);
		Ray diffuse_ray(hit.pos + diffuse_dir * RAY_REFLECT_NUDGE_MULTIPLIER, diffuse_dir);
		float cosi = Vec3Dot(diffuse_dir, hit.normal);
		Vec4 irradiance = cosi * TracePath(diffuse_ray, ray_depth + 1);

		Vec3 diffuse_brdf = hit.mat->albedo * INV_PI;
		final_color.xyz += 2.0f * PI * diffuse_brdf * irradiance.xyz;
	}

	return final_color;
}

void Render()
{
	if (data.pause_rendering)
	{
		return;
	}

	UVec2 framebuffer_size = Window::GetFramebufferSize();
	UVec2 job_size(16, 16);
	Vec2 inv_framebuffer_size(1.0f / framebuffer_size.x, 1.0f / framebuffer_size.y);

	data.num_accumulated++;

	auto trace_ray_thread_job = [&job_size, &framebuffer_size, &inv_framebuffer_size](ThreadPool::JobDispatchArgs args) {
		uint32_t first_x = (args.job_index * job_size.x) % framebuffer_size.x;
		uint32_t first_y = ((args.job_index * job_size.x) / framebuffer_size.x) * job_size.y;

		for (uint32_t y = first_y; y < first_y + job_size.y; y += 4)
			for (uint32_t x = first_x; x < first_x + job_size.x; x += 4)
				for (uint32_t v = 0; v < 4; ++v)
					for (uint32_t u = 0; u < 4; ++u)
					{
						const float screen_u = (float)(x + u) * inv_framebuffer_size.x;
						const float screen_v = (float)(y + v) * inv_framebuffer_size.y;

						Ray ray = data.camera.GetRay(screen_u, screen_v);
						Vec4 final_color = Vec4(0.0f);

						if (data.render_mode == RENDER_MODE_COMPARISON)
						{
							if (x < (framebuffer_size.x / 2))
								final_color = TracePath(ray, 0);
							else
								final_color = TracePathNEE(ray, 0, false);
						}
						else if (data.render_mode == RENDER_MODE_BRUTE_FORCE)
						{
							final_color = TracePath(ray, 0);
						}
						else if (data.render_mode == RENDER_MODE_NEXT_EVENT_EST)
						{
							final_color = TracePathNEE(ray, 0, false);
						}

						data.total_energy_received += final_color.x + final_color.y + final_color.z;
						uint32_t framebuffer_pos = (y + v) * framebuffer_size.x + (x + u);

						data.accumulator[framebuffer_pos] += final_color;
						data.pixels[framebuffer_pos] = Vec4ToUint(data.accumulator[framebuffer_pos] / data.num_accumulated);
					}
	};

	size_t pixel_count = framebuffer_size.x * framebuffer_size.y;
	uint32_t num_jobs = pixel_count / (job_size.y * job_size.x);

	ThreadPool::Dispatch(num_jobs, 16, trace_ray_thread_job);
	ThreadPool::WaitAll();
}

int main(int argc, char* argv[])
{
	Window::CreateWindowArgs window_args = {};
	window_args.width = 1280;
	window_args.height = 720;
	window_args.title = L"Graphics Advanced Masterclass";
	Window::Create(window_args);

	UVec2 framebuffer_size = Window::GetFramebufferSize();

	DX12::DX12InitArgs dx12_args = {};
	dx12_args.hWnd = Window::GetHandle();
	dx12_args.width = framebuffer_size.x;
	dx12_args.height = framebuffer_size.y;
	DX12::Init(dx12_args);

	ThreadPool::Init();

	data.pixels.resize(framebuffer_size.x * framebuffer_size.y);
	data.accumulator.resize(framebuffer_size.x * framebuffer_size.y);
	data.camera = Camera(Vec3(0.0f, 0.0f, 8.0f), Vec3(0.0f, 0.0f, -1.0f), 60.0f, (float)framebuffer_size.x / framebuffer_size.y);

	data.materials.emplace_back(Vec3(0.2f, 0.2f, 0.8f), 0.0f);
	data.materials.emplace_back(Vec3(0.4f), 0.0f);
	data.materials.emplace_back(Vec3(1.0f, 0.95f, 0.8f), 10.0f, true);
	data.materials.emplace_back(Vec3(1.0f), 0.1f, 0.9f, Vec3(0.2f, 0.8f, 0.8f), 1.517f);

	// Load mesh and build its BVH
	Mesh dragon_mesh = GLTFLoader::Load("Assets/Models/Dragon/DragonAttenuation.gltf");
	//Mesh dragon_mesh = GLTFLoader::Load("Assets/Models/Cube/Cube.gltf");
	data.objects.emplace_back("Dragon", dragon_mesh, 0, BVH::BuildOption_SAHSplitIntervals);

	Mesh ground_mesh;
	ground_mesh.indices.push_back(0);
	ground_mesh.indices.push_back(1);
	ground_mesh.indices.push_back(2);
	ground_mesh.indices.push_back(2);
	ground_mesh.indices.push_back(3);
	ground_mesh.indices.push_back(0);
	ground_mesh.vertices.push_back({ Vec3(-1000.0f, -3.0f, 1000.0f), Vec3(0.0f, 1.0f, 0.0f) });
	ground_mesh.vertices.push_back({ Vec3(-1000.0f, -3.0f, -1000.0f), Vec3(0.0f, 1.0f, 0.0f) });
	ground_mesh.vertices.push_back({ Vec3(1000.0f, -3.0f, -1000.0f), Vec3(0.0f, 1.0f, 0.0f) });
	ground_mesh.vertices.push_back({ Vec3(1000.0f, -3.0f, 1000.0f), Vec3(0.0f, 1.0f, 0.0f) });
	data.objects.emplace_back("Ground", ground_mesh, 1, BVH::BuildOption_SAHSplitIntervals);

	/*Mesh light_mesh;
	light_mesh.indices.push_back(0);
	light_mesh.indices.push_back(1);
	light_mesh.indices.push_back(2);
	light_mesh.indices.push_back(2);
	light_mesh.indices.push_back(3);
	light_mesh.indices.push_back(0);
	light_mesh.vertices.push_back({ Vec3(-10.0f, 20.0f, 10.0f), Vec3(0.0f, -1.0f, 0.0f) });
	light_mesh.vertices.push_back({ Vec3(-10.0f, 20.0f, -10.0f), Vec3(0.0f, -1.0f, 0.0f) });
	light_mesh.vertices.push_back({ Vec3(10.0f, 20.0f, -10.0f), Vec3(0.0f, -1.0f, 0.0f) });
	light_mesh.vertices.push_back({ Vec3(10.0f, 20.0f, 10.0f), Vec3(0.0f, -1.0f, 0.0f) });
	data.objects.emplace_back("Light", light_mesh, 2, BVH::BuildOption_SAHSplitIntervals);
	data.light_source_indices.emplace_back(data.objects.size() - 1);*/

	data.objects.emplace_back("Spherical light0", Primitive(Sphere(Vec3(10.0f, 10.0f, 10.0f), 5.0f)), 2);
	data.light_source_indices.emplace_back(data.objects.size() - 1);
	data.objects.emplace_back("Spherical light1", Primitive(Sphere(Vec3(-10.0f, 10.0f, -10.0f), 5.0f)), 2);
	data.light_source_indices.emplace_back(data.objects.size() - 1);

	std::chrono::high_resolution_clock::time_point curr_time = std::chrono::high_resolution_clock::now(),
		last_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<float> delta_time = std::chrono::duration<float>(0.0f);

	while (!Window::ShouldClose())
	{
		curr_time = std::chrono::high_resolution_clock::now();
		delta_time = curr_time - last_time;

		ImGui_ImplWin32_NewFrame();
		ImGui_ImplDX12_NewFrame();
		ImGui::NewFrame();

		Window::PollEvents();
		Update(delta_time.count());
		Render();

		ImGui::Begin("General");
		ImGui::Text("Accumulated frames: %u", data.num_accumulated);
		if (ImGui::Checkbox("Pause rendering", &data.pause_rendering))
		{
			ResetAccumulator();
		}
		ImGui::Text("Frame time (CPU): %.3f ms", delta_time.count() * 1000.0f);
		ImGui::Text("Traced ray count: %u", data.stats.traced_rays);
		ImGui::Text("Total energy received: %.3f", data.total_energy_received / data.num_accumulated);
		ImGui::SliderInt("Max ray depth", &data.max_ray_depth, 1, 8);
		if (ImGui::BeginCombo("Render mode", render_mode_labels[data.render_mode]))
		{
			for (size_t i = 0; i < RENDER_MODE_NUM_MODES; ++i)
			{
				bool is_selected = i == data.render_mode;
				if (ImGui::Selectable(render_mode_labels[i], is_selected))
				{
					data.render_mode = (RenderMode)i;
					ResetAccumulator();
				}

				if (is_selected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}

			ImGui::EndCombo();
		}
		if (ImGui::BeginCombo("Debug render view", debug_render_mode_labels[data.debug_render_mode]))
		{
			for (size_t i = 0; i < DEBUG_RENDER_MODE_NUM_MODES; ++i)
			{
				bool is_selected = i == data.debug_render_mode;
				if (ImGui::Selectable(debug_render_mode_labels[i], is_selected))
				{
					data.debug_render_mode = (DebugRenderMode)i;
				}

				if (is_selected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}

			ImGui::EndCombo();
		}

		ImGui::SetNextItemOpen(true, ImGuiCond_Once);
		if (ImGui::CollapsingHeader("Scene"))
		{
			ImGui::Indent(10.0f);

			for (auto& obj : data.objects)
			{
				if (ImGui::CollapsingHeader(obj.name))
				{
					ImGui::PushID(obj.name);
					ImGui::Indent(10.0f);

					obj.RenderImGui();

					ImGui::Unindent(10.0f);
					ImGui::PopID();
				}
			}

			ImGui::Unindent(10.0f);
		}
		ImGui::End();

		DX12::CopyToBackBuffer(data.pixels.data(), data.pixels.size() * sizeof(uint32_t));
		DX12::Present();

		ImGui::EndFrame();

		last_time = curr_time;
		data.stats.Reset();
	}

	ThreadPool::Exit();
	DX12::Exit();
	Window::Destroy();

	return 0;
}