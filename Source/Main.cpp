#include "DX12.h"
#include "MathLib.h"
#include "Window.h"
#include "Input.h"
#include "ThreadPool.h"
#include "Primitives.h"
#include "Random.h"
#include "BVH.h"

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

#include <stdint.h>
#include <stdexcept>
#include <chrono>
#include <array>
#include <vector>
#include <unordered_map>
#include <algorithm>

#include "imgui/imgui.h"
#include "imgui/imgui_impl_win32.h"
#include "imgui/imgui_impl_dx12.h"

/*
	
	Whitted-style raytracer (Finished):
	- Direct illumination of multiple light sources, takes into account:
		- Visibility
		- Distance attenuation
		- Shading model (N dot L for diffuse, or lambert)
	- Pure specular reflections with recursion
	- Dielectrics with fresnel with recursion
	- Beer's Law

	Cook raytracer (In-progress):
	- Area lights
	- Glossy reflections
	- Anti-aliasing
	- Motion blur
	- Depth of field

*/

static constexpr uint8_t MAX_RAY_DEPTH = 5;
static constexpr float RAY_REFLECT_NUDGE_MULTIPLIER = 0.001f;

struct Material
{
	Vec4 albedo = Vec4(0.0f);
	float specular = 0.0f;
	float refractivity = 0.0f;
	Vec3 absorption = Vec3(0.0f);
	float ior = 1.0f;
};

struct PointLight
{
	Vec3 position = Vec3(0.0f);
	Vec3 color = Vec3(1.0f);
	float intensity = 1.0f;
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

	void Update(float dt)
	{
		//Vec2 mouse_move = Input::GetMouseMoveRel();

		m_pos.x -= Input::GetInputAxis1D(Input::KeyCode_A, Input::KeyCode_D) * dt * m_speed;
		m_pos.y += Input::GetInputAxis1D(Input::KeyCode_Space, Input::KeyCode_LeftShift) * dt * m_speed;
		m_pos.z -= Input::GetInputAxis1D(Input::KeyCode_W, Input::KeyCode_S) * dt * m_speed;

		UpdateScreenPlane();
	}

	Ray GetRay(float u, float v)
	{
		Vec3 pixel_pos = m_screen_plane.top_left +
			u * (m_screen_plane.top_right - m_screen_plane.top_left) +
			v * (m_screen_plane.bottom_left - m_screen_plane.top_left);

		return { .origin = m_pos, .direction = Vec3Normalize(pixel_pos - m_pos) };
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

enum DebugRenderView
{
	DebugRenderView_None,
	DebugRenderView_Diffuse,
	DebugRenderView_Specular,
	DebugRenderView_Refract,
	DebugRenderView_SurfaceNormal,
	DebugRenderView_SurfaceAlbedo,
	DebugRenderView_DirectIllumination,
	DebugRenderView_Depth,
	DebugRenderView_ViewDirection,
	DebugRenderView_BVHDepth,
	DebugRenderView_NumViews
};

std::vector<const char*> debug_render_view_names =
{
	{ "None", "Diffuse", "Specular", "Refract", "Surface normal", "Surface albedo", "Direct illumination", "Depth", "View direction", "BVH Depth" }
};

struct Data
{
	// Render data
	std::vector<uint32_t> pixels;
	std::vector<Material> materials;
	std::vector<Plane> planes;
	std::vector<Sphere> spheres;
	std::vector<Triangle> triangles;
	std::vector<PointLight> point_lights;

	BVH bounding_volume_hierarchy;

	Camera camera;

	DebugRenderView debug_view = DebugRenderView_None;
} static data;

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

	data.camera.Update(dt);
}

void IntersectScene(Ray& ray)
{
	data.bounding_volume_hierarchy.Traverse(ray);
}

bool TraceShadowRay(Ray& shadow_ray)
{
	IntersectScene(shadow_ray);
	return shadow_ray.payload.object_ptr != nullptr;
}

Vec3 GetDirectIlluminationAtPoint(const Vec3& point, const Vec3& normal)
{
	for (const auto& point_light : data.point_lights)
	{
		float distance_to_light = Vec3Length(point_light.position - point);
		Vec3 light_dir = Vec3Normalize(point_light.position - point);

		Ray shadow_ray = { .origin = point + light_dir * RAY_REFLECT_NUDGE_MULTIPLIER, .direction = light_dir, .t = distance_to_light };
		bool occluded = TraceShadowRay(shadow_ray);

		if (occluded)
		{
			return Vec3(0.0f, 0.0f, 0.0f);
		}

		float NoL = std::max(0.0f, Vec3Dot(normal, light_dir));
		float attenuation = 1.0f / (distance_to_light * distance_to_light);
		Vec3 radiance = point_light.color * point_light.intensity * attenuation;
		Vec3 indicent_light = radiance * NoL;

		return indicent_light;
	}
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

Vec4 TraceRay(Ray& ray, uint8_t depth)
{
	if (depth == MAX_RAY_DEPTH)
	{
		return Vec4(0.0f);
	}

	uint8_t next_depth = depth + 1;

	IntersectScene(ray);

	if (!ray.payload.object_ptr)
	{
		return Vec4(0.5f, 0.8f, 1.0f, 1.0f);
	}

	Vec4 final_color(0.0f, 0.0f, 0.0f, 1.0f);

	Vec3 surface_point = ray.origin + ray.t * ray.direction;
	Vec3 surface_normal = GetObjectSurfaceNormalAtPoint(ray.payload.object_type, ray.payload.object_ptr, surface_point);
	Material& surface_material = data.materials[ray.payload.mat_index];

	Vec3 illumination = GetDirectIlluminationAtPoint(surface_point, surface_normal);
	Vec3 ambient(0.2f);

	Vec3 diffuse = surface_material.albedo.xyz * INV_PI * (illumination + ambient) * (1.0f - surface_material.specular - surface_material.refractivity);
	Vec3 specular(0.0f);
	Vec3 refract(0.0f);

	if (surface_material.specular > 0.0f)
	{
		Vec3 reflect_dir = Reflect(ray.direction, surface_normal);
		Ray reflect_ray = { .origin = surface_point + reflect_dir * RAY_REFLECT_NUDGE_MULTIPLIER, .direction = reflect_dir };
		Vec4 reflect_color = TraceRay(reflect_ray, next_depth);
		specular += surface_material.albedo.xyz * reflect_color.xyz * surface_material.specular;
	}

	if (surface_material.refractivity > 0.0f)
	{
		Vec3 reflect_dir = Reflect(ray.direction, surface_normal);
		Ray reflect_ray = { .origin = surface_point + reflect_dir * RAY_REFLECT_NUDGE_MULTIPLIER, .direction = reflect_dir };

		Vec3 N = surface_normal;
		float cosi = std::clamp(Vec3Dot(N, ray.direction), -1.0f, 1.0f);
		float etai = 1.0f, etat = surface_material.ior;
		bool inside = true;

		if (cosi < 0.0f)
		{
			cosi = -cosi;
			inside = false;
		}
		else
		{
			N = -surface_normal;
			std::swap(etai, etat);
		}

		float eta = etai / etat;
		float k = 1.0f - eta * eta * (1.0f - cosi * cosi);
		float Fr = 1.0f;

		if (k >= 0.0f)
		{
			Vec3 refract_dir = Vec3Normalize(ray.direction * eta + ((eta * cosi - std::sqrtf(k)) * N));
			Ray refract_ray = { .origin = surface_point + refract_dir * RAY_REFLECT_NUDGE_MULTIPLIER, .direction = refract_dir };

			float angle_in = Vec3Dot(ray.direction, surface_normal);
			float angle_out = Vec3Dot(refract_dir, surface_normal);

			Fr = Fresnel(angle_in, angle_out, etai, etat);
			refract += TraceRay(refract_ray, next_depth).xyz * (1.0f - Fr);

			Vec3 absorption(0.0f);
			absorption.x = std::expf(-surface_material.absorption.x * ray.t);
			absorption.y = std::expf(-surface_material.absorption.y * ray.t);
			absorption.z = std::expf(-surface_material.absorption.z * ray.t);

			if (inside)
			{
				refract *= absorption;
			}
		}

		Vec4 reflect_color = TraceRay(reflect_ray, next_depth);
		specular += surface_material.albedo.xyz * Fr * reflect_color.xyz;
	}

	final_color.xyz = diffuse + specular + refract;

	if (depth == 0)
	{
		switch (data.debug_view)
		{
		case DebugRenderView_Diffuse:
		{
			final_color.xyz = diffuse;
		} break;
		case DebugRenderView_Specular:
		{
			final_color.xyz = specular;
		} break;
		case DebugRenderView_Refract:
		{
			final_color.xyz = refract;
		} break;
		case DebugRenderView_SurfaceNormal:
		{
			final_color.xyz = (surface_normal + 1.0f) * 0.5f;
		} break;
		case DebugRenderView_SurfaceAlbedo:
		{
			final_color.xyz = surface_material.albedo.xyz;
		} break;
		case DebugRenderView_DirectIllumination:
		{
			final_color.xyz = illumination;
		} break;
		case DebugRenderView_Depth:
		{
			final_color.xyz = 0.1f * ray.t;
		} break;
		case DebugRenderView_ViewDirection:
		{
			final_color.xyz = (ray.direction + 1.0f) * 0.5f;
		} break;
		case DebugRenderView_BVHDepth:
		{
			// Lerp from white to dark red based on bvh depth
			final_color.xyz = Vec3Lerp(Vec3(1.0f), Vec3(1.0f, 0.0f, 0.0f), std::min(1.0f, (float)ray.payload.bvh_depth / 20.0f));
		} break;
		}
	}

	return final_color;
}

void Render()
{
	UVec2 framebuffer_size = Window::GetFramebufferSize();
	UVec2 job_size(16, 16);
	Vec2 inv_framebuffer_size(1.0f / framebuffer_size.x, 1.0f / framebuffer_size.y);

	auto trace_ray_thread_job = [&job_size, &framebuffer_size, &inv_framebuffer_size](ThreadPool::JobDispatchArgs args) {
		uint32_t first_x = (args.job_index * job_size.x) % framebuffer_size.x;
		uint32_t first_y = ((args.job_index * job_size.x) / framebuffer_size.x) * job_size.y;

		for (uint32_t y = first_y; y < first_y + job_size.y; ++y)
		{
			for (uint32_t x = first_x; x < first_x + job_size.x; ++x)
			{
				const float u = (float)x * inv_framebuffer_size.x;
				const float v = (float)y * inv_framebuffer_size.y;
				
				Ray ray = data.camera.GetRay(u, v);
				Vec4 final_color = TraceRay(ray, 0);

				data.pixels[y * framebuffer_size.x + x] = Vec4ToUint(final_color);
			}
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

	data.camera = Camera(Vec3(0.0f), Vec3(0.0f, 0.0f, -1.0f), 60.0f, (float)framebuffer_size.x / framebuffer_size.y);
	data.pixels.resize(framebuffer_size.x * framebuffer_size.y);

	data.materials.emplace_back(Vec4(0.3f, 0.3f, 0.3f, 1.0f), 0.0f, 0.0f, Vec3(0.0f), 1.0f);
	data.materials.emplace_back(Vec4(0.9f, 0.3f, 0.3f, 1.0f), 0.0f, 0.0f, Vec3(0.0f), 1.0f);
	data.materials.emplace_back(Vec4(0.3f, 0.3f, 0.9f, 1.0f), 0.0f, 0.0f, Vec3(0.0f), 1.0f);
	data.materials.emplace_back(Vec4(0.2f, 0.2f, 0.2f, 1.0f), 0.0f, 0.8f, Vec3(0.9f, 0.2f, 0.3f), 1.517f);

	data.planes.emplace_back(Vec3(0.0f, 1.0f, 0.0f), Vec3(0.0f, -5.0f, 0.0f), 0);
	/*data.planes.emplace_back(Vec3(0.0f, -1.0f, 0.0f), Vec3(0.0f, 5.0f, 0.0f), 0);
	data.planes.emplace_back(Vec3(1.0f, 0.0f, 0.0f), Vec3(-5.0f, 0.0f, 0.0f), 1);
	data.planes.emplace_back(Vec3(-1.0f, 0.0f, 0.0f), Vec3(5.0f, 0.0f, 0.0f), 1);
	data.planes.emplace_back(Vec3(0.0f, 0.0f, 1.0f), Vec3(0.0f, 0.0f, -5.0f), 2);
	data.planes.emplace_back(Vec3(0.0f, 0.0f, -1.0f), Vec3(0.0f, 0.0f, 5.0f), 2);

	data.spheres.emplace_back(Vec3(-2.0f, 0.0f, -2.0f), 1.0f * 1.0f, 3);
	data.spheres.emplace_back(Vec3(0.0f, 0.0f, -2.0f), 0.5f * 0.5f, 3);
	data.spheres.emplace_back(Vec3(2.0f, 0.0f, -2.0f), 0.25f * 0.25f, 3);*/

	for (uint32_t i = 0; i < 64; ++i)
	{
		Vec3 v0 = RandomVec3(-4.0f, 4.0f);
		Vec3 v1 = RandomVec3();
		Vec3 v2 = RandomVec3();
		v1 = v0 + v1;
		v2 = v0 + v2;
		data.triangles.emplace_back(v0, v1, v2, RandomUInt32(0, 2));
	}
	//data.triangles.emplace_back(Vec3(0.0f, 2.0f, -1.0f), Vec3(2.0f, 0.0f, -1.0f), Vec3(-2.0f, 0.0f, -1.0f), 0);
	data.bounding_volume_hierarchy.Build(data.triangles);

	data.point_lights.emplace_back(Vec3(0.0f, 4.5f, 0.0f), Vec3(1.0f), 200.0f);

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
		ImGui::Text("Frame time (CPU): %.3f ms", delta_time.count() * 1000.0f);
		if (ImGui::BeginCombo("Debug render view", debug_render_view_names[data.debug_view]))
		{
			for (size_t i = 0; i < DebugRenderView_NumViews; ++i)
			{
				bool is_selected = i == data.debug_view;

				if (ImGui::Selectable(debug_render_view_names[i], is_selected))
				{
					data.debug_view = (DebugRenderView)i;
				}

				if (is_selected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}

			ImGui::EndCombo();
		}
		ImGui::End();

		DX12::CopyToBackBuffer(data.pixels.data(), data.pixels.size() * sizeof(uint32_t));
		DX12::Present();

		ImGui::EndFrame();

		last_time = curr_time;
	}

	ThreadPool::Exit();
	DX12::Exit();
	Window::Destroy();

	return 0;
}