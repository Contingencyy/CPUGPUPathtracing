#include "DX12.h"
#include "MathLib.h"
#include "Window.h"
#include "Input.h"
#include "ThreadPool.h"

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
	
	Whitted-style raytracer:
	Done:
	- Direct illumination of multiple light sources, takes into account:
		- Visibility
		- Distance attenuation
		- Shading model (N dot L for diffuse, or lambert)
	- Pure specular reflections with recursion

	Todo:
	- Dielectrics with fresnel with recursion
	- Beer's Law

*/

static constexpr uint8_t MAX_RAY_DEPTH = 5;
static constexpr float RAY_REFLECT_NUDGE_MULTIPLIER = 0.001f;

enum ObjectType : uint8_t
{
	ObjectType_Plane,
	ObjectType_Sphere,
	ObjectType_NumTypes
};

struct Ray
{
	Vec3 origin = Vec3(0.0f);
	Vec3 direction = Vec3(0.0f);
	float t = 1e34f;
	bool inside = false;

	struct Payload
	{
		ObjectType object_type;
		void* object_ptr;
		uint32_t mat_index;
	} payload;
};

struct Plane
{
	Vec3 normal = Vec3(0.0f);
	Vec3 point = Vec3(0.0f);

	uint32_t mat_index = 0;
};

struct Sphere
{
	Vec3 center = Vec3(0.0f);
	float radius_sq = 0.0f;

	uint32_t mat_index = 0;
};

struct Material
{
	Vec4 albedo = Vec4(0.0f);
	float specular = 0.0f;
	float refractivity = 0.0f;
	Vec3 absorption = Vec3(0.0f);
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
	DebugRenderView_NumViews
};

std::vector<const char*> debug_render_view_names =
{
	{ "None", "Diffuse", "Specular", "Refract", "Surface normal", "Surface albedo", "Direct illumination", "Depth", "View direction" }
};

struct Data
{
	// Render data
	std::vector<uint32_t> pixels;
	std::vector<Material> materials;
	std::vector<Plane> planes;
	std::vector<Sphere> spheres;
	std::vector<PointLight> point_lights;

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

	static float t = 0.0f;
	t += dt;
	for (auto& sphere : data.spheres)
	{
		sphere.center.y = cosf(t);
	}
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
			ray.payload.object_type = ObjectType_Plane;
			ray.payload.object_ptr = (void*)&plane;
			ray.payload.mat_index = plane.mat_index;
		}
	}
}

void IntersectSphere(const Sphere& sphere, Ray& ray)
{
	float t0, t1;
#if 1
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
#else
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
		ray.payload.object_type = ObjectType_Sphere;
		ray.payload.object_ptr = (void*)&sphere;
		ray.payload.mat_index = sphere.mat_index;
	}
}

Vec3 GetObjectSurfaceNormalAtPoint(ObjectType type, void* ptr, const Vec3& point)
{
	switch (type)
	{
	case ObjectType_Plane:
	{
		Plane* plane = reinterpret_cast<Plane*>(ptr);
		return plane->normal;
	} break;
	case ObjectType_Sphere:
	{
		Sphere* sphere = reinterpret_cast<Sphere*>(ptr);
		return Vec3Normalize(point - sphere->center);
	} break;
	}
}

bool TraceShadowRay(Ray& shadow_ray)
{
	for (const auto& plane : data.planes)
	{
		IntersectPlane(plane, shadow_ray);
	}

	for (const auto& sphere : data.spheres)
	{
		IntersectSphere(sphere, shadow_ray);
	}

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

	for (const auto& plane : data.planes)
	{
		IntersectPlane(plane, ray);
	}
	
	for (const auto& sphere : data.spheres)
	{
		IntersectSphere(sphere, ray);
	}

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

		float cosi = std::clamp(Vec3Dot(ray.direction, surface_normal), -1.0f, 1.0f);
		float etai = 1.0f, etat = 1.2f;
		Vec3 N = surface_normal;
		Vec3 absorption(0.0f);

		float Fr = 0.0f;
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
			absorption.x = std::expf(-surface_material.absorption.x * ray.t);
			absorption.y = std::expf(-surface_material.absorption.y * ray.t);
			absorption.z = std::expf(-surface_material.absorption.z * ray.t);
		}

		float eta = etai / etat;
		float k = 1.0f - eta * eta * (1.0f - cosi * cosi);

		if (k >= 0.0f)
		{
			Vec3 refract_dir = Vec3Normalize(ray.direction * eta + ((eta * cosi - std::sqrtf(k)) * N));
			Ray refract_ray = { .origin = surface_point + refract_dir * RAY_REFLECT_NUDGE_MULTIPLIER, .direction = refract_dir };

			float angle_in = Vec3Dot(ray.direction, surface_normal);
			float angle_out = Vec3Dot(refract_dir, surface_normal);

			Fr = Fresnel(angle_in, angle_out, etai, etat);
			refract += TraceRay(refract_ray, next_depth).xyz * (1.0f - Fr);

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
	data.materials.emplace_back(Vec4(0.8f, 0.8f, 0.8f, 1.0f), 0.5f, 0.0f, Vec3(0.0f));
	data.materials.emplace_back(Vec4(0.3f, 0.2f, 0.2f, 1.0f), 0.0f, 0.8f, Vec3(0.0f, 0.9f, 0.9f));
	data.planes.emplace_back(Vec3(0.0f, 1.0f, 0.0f), Vec3(0.0f, -2.0f, 0.0f), 0);
	data.spheres.emplace_back(Vec3(-2.0f, 0.0f, -2.0f), 1.0f * 1.0f, 1);
	data.spheres.emplace_back(Vec3(0.0f, 0.0f, -2.0f), 0.5f * 0.5f, 1);
	data.spheres.emplace_back(Vec3(2.0f, 0.0f, -2.0f), 0.25f * 0.25f, 1);
	data.point_lights.emplace_back(Vec3(0.0f, 2.0f, 2.0f), Vec3(1.0f), 100.0f);

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