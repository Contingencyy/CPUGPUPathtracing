#include "DX12.h"
#include "MathLib.h"
#include "Window.h"
#include "Input.h"

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

#include "imgui/imgui.h"
#include "imgui/imgui_impl_win32.h"
#include "imgui/imgui_impl_dx12.h"

/*
	
	TODOS:
	- Shadow rays
	- Reflection rays
	- Refraction rays
	- Anti-aliasing
	- Multi-threading
	- Beers law
	- Motion blur
	- Depth of field

*/

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

struct Data
{
	// Render data
	std::vector<uint32_t> pixels;
	std::vector<Plane> planes;
	std::vector<Sphere> spheres;
	std::vector<Material> materials;

	Camera camera;
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

void IntersectPlane(const Plane& plane, Ray& ray)
{
	// Plane: P * N + d = 0
	// Ray: P(t) = O + tD

	float angle = Vec3Dot(ray.direction, plane.normal);

	if (angle > 1e-6)
	{
		Vec3 p0 = plane.point - ray.origin;
		float t = Vec3Dot(p0, plane.normal) / angle;

		if (t < ray.t)
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

	ray.t = t1;
	ray.payload.object_type = ObjectType_Sphere;
	ray.payload.object_ptr = (void*)&sphere;
	ray.payload.mat_index = sphere.mat_index;
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

Vec3 GetDirectIlluminationAtPoint(const PointLight& point_light, const Vec3& point, const Vec3& normal)
{
	float distance_to_light = Vec3Length(point_light.position - point);
	Vec3 light_dir = Vec3Normalize(point - point_light.position);

	float NoL = std::max(0.0f, Vec3Dot(normal, Vec3(light_dir.x, light_dir.y, light_dir.z)));
	float attenuation = 1.0f / (distance_to_light * distance_to_light);
	Vec3 radiance = point_light.color * point_light.intensity * attenuation;
	Vec3 indicent_light = radiance * NoL;

	return indicent_light;
}

Vec4 TraceRay(Ray& ray)
{
	for (auto& plane : data.planes)
	{
		IntersectPlane(plane, ray);
	}
	
	for (auto& sphere : data.spheres)
	{
		IntersectSphere(sphere, ray);
	}

	if (!ray.payload.object_ptr)
	{
		return Vec4(1.0f, 0.0f, 1.0f, 1.0f);
	}

	Vec4 final_color(0.0f, 0.0f, 0.0f, 1.0f);

	Vec3 surface_point = ray.origin + ray.t * ray.direction;
	Vec3 surface_normal = GetObjectSurfaceNormalAtPoint(ray.payload.object_type, ray.payload.object_ptr, surface_point);
	Material& surface_material = data.materials[ray.payload.mat_index];

	PointLight point_light = { Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f), 10.0f };
	Vec3 illumination = GetDirectIlluminationAtPoint(point_light, surface_point, surface_normal);

	// Lambertian diffuse
	final_color.xyz = surface_material.albedo.xyz * INV_PI * illumination;
	return final_color;
}

void Render()
{
	UVec2 framebuffer_size = Window::GetFramebufferSize();

	for (uint32_t y = 0; y < framebuffer_size.y; ++y)
	{
		for (uint32_t x = 0; x < framebuffer_size.x; ++x)
		{
			const float u = (float)x * (1.0f / framebuffer_size.x);
			const float v = (float)y * (1.0f / framebuffer_size.y);

			Ray ray = data.camera.GetRay(u, v);
			Vec4 final_color = TraceRay(ray);

			data.pixels[y * framebuffer_size.x + x] = Vec4ToUint(final_color);
		}
	}
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

	data.camera = Camera(Vec3(0.0f), Vec3(0.0f, 0.0f, -1.0f), 60.0f, (float)framebuffer_size.x / framebuffer_size.y);

	data.pixels.resize(framebuffer_size.x * framebuffer_size.y);
	data.materials.emplace_back(Vec4(0.5f, 0.35f, 0.25f, 1.0f));
	data.materials.emplace_back(Vec4(0.1f, 0.25f, 0.8f, 1.0f));
	data.planes.emplace_back(Vec3(0.0f, -1.0f, 0.0f), Vec3(0.0f, -1.0f, 0.0f), 0);
	data.spheres.emplace_back(Vec3(0.0f, 0.0f, -2.0f), 1.0f * 1.0f, 1);

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
		ImGui::End();

		DX12::CopyToBackBuffer(data.pixels.data(), data.pixels.size() * sizeof(uint32_t));
		DX12::Present();

		ImGui::EndFrame();

		last_time = curr_time;
	}

	DX12::Exit();

	Window::Destroy();

	return 0;
}