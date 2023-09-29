#include "DX12.h"
#include "MathLib.h"

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
#include <vector>

enum ObjectType : uint8_t
{
	ObjectType_Plane,
	ObjectType_Sphere
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

struct Data
{
	// Window data
	HWND hWnd = nullptr;
	uint32_t window_width = 1280;
	uint32_t window_height = 720;

	// Application data
	bool should_close = false;

	// Render data
	std::vector<uint32_t> pixels;
	std::vector<Plane> planes;
	std::vector<Sphere> spheres;
	std::vector<Material> materials;
} static data;

LRESULT CALLBACK WindowEventCallback(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	switch (message)
	{
	case WM_DESTROY:
	{
		::PostQuitMessage(0);
	} break;
	default:
	{
		return ::DefWindowProcW(hWnd, message, wParam, lParam);
	} break;
	}

	return 0;
}

struct CreateWindowArgs
{
	uint32_t width = 1280;
	uint32_t height = 720;
	const wchar_t* title = L"DefaultWindowTitle";
};

void CreateWindow(const CreateWindowArgs& args)
{
	// Register window class
	HINSTANCE hInst = GetModuleHandle(NULL);
	const wchar_t* windowClassName = L"DefaultWindowClass";

	WNDCLASSEXW windowClass = {};
	windowClass.cbSize = sizeof(WNDCLASSEX);
	windowClass.style = CS_HREDRAW | CS_VREDRAW;
	windowClass.lpfnWndProc = &WindowEventCallback;
	windowClass.cbClsExtra = 0;
	windowClass.cbWndExtra = 0;
	windowClass.hInstance = hInst;
	windowClass.hIcon = ::LoadIcon(hInst, "");
	windowClass.hCursor = ::LoadCursor(NULL, IDC_ARROW);
	windowClass.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
	windowClass.lpszMenuName = NULL;
	windowClass.lpszClassName = windowClassName;
	windowClass.hIconSm = ::LoadIcon(hInst, NULL);

	ATOM atom = ::RegisterClassExW(&windowClass);
	if (atom == 0)
	{
		throw std::runtime_error("Failed to register window class");
	}

	// Create window
	int screen_width = ::GetSystemMetrics(SM_CXSCREEN);
	int screen_height = ::GetSystemMetrics(SM_CYSCREEN);

	RECT window_rect = { 0, 0, static_cast<LONG>(args.width), static_cast<LONG>(args.height) };
	::AdjustWindowRect(&window_rect, WS_OVERLAPPEDWINDOW, FALSE);

	int window_width = window_rect.right - window_rect.left;
	int window_height = window_rect.bottom - window_rect.top;

	int window_x = std::max(0, (screen_width - window_width) / 2);
	int window_y = std::max(0, (screen_height - window_height) / 2);

	data.hWnd = ::CreateWindowExW(NULL, windowClassName, args.title, WS_OVERLAPPEDWINDOW,
		window_x, window_y, window_width, window_height, NULL, NULL, hInst, nullptr);
	
	if (!data.hWnd)
	{
		throw std::runtime_error("Failed to create window");
	}

	RECT client_rect = {};
	::GetClientRect(data.hWnd, &client_rect);
	data.window_width = client_rect.right - client_rect.left;
	data.window_height = client_rect.bottom - client_rect.top;
	::ShowWindow(data.hWnd, 1);
}

void DestroyWindow()
{
	DestroyWindow(data.hWnd);
}

void PollEvents()
{
	MSG msg = {};
	while (::PeekMessage(&msg, NULL, 0, 0, PM_REMOVE) != NULL)
	{
		if (msg.message == WM_QUIT)
		{
			data.should_close = true;
			break;
		}

		::TranslateMessage(&msg);
		::DispatchMessage(&msg);
	}
}

void Update(float dt)
{
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

Material& GetObjectSurfaceMaterial(ObjectType type, void* ptr)
{
	switch (type)
	{
	case ObjectType_Plane:
	{
		Plane* plane = reinterpret_cast<Plane*>(ptr);
		return data.materials[plane->mat_index];
	} break;
	case ObjectType_Sphere:
	{
		Sphere* sphere = reinterpret_cast<Sphere*>(ptr);
		return data.materials[sphere->mat_index];
	} break;
	}
}

Vec3 GetDirectIlluminationAtPoint(const PointLight& point_light, const Vec3& point, const Vec3& normal)
{
	float distance_to_light = Vec3Length(point_light.position - point);
	Vec3 light_dir = Vec3Normalize(point_light.position - point);

	float NoL = std::max(0.0f, Vec3Dot(normal, Vec3(-light_dir.x, -light_dir.y, -light_dir.z)));
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
	Material& surface_material = GetObjectSurfaceMaterial(ray.payload.object_type, ray.payload.object_ptr);

	PointLight point_light = { Vec3(0.0f, 0.0f, 0.0f), Vec3(1.0f), 5.0f };
	Vec3 illumination = GetDirectIlluminationAtPoint(point_light, surface_point, surface_normal);

	final_color.x = surface_material.albedo.x * illumination.x;
	final_color.y = surface_material.albedo.y * illumination.y;
	final_color.z = surface_material.albedo.z * illumination.z;

	return final_color;
}

void Render()
{
	float aspect = (float)data.window_width / data.window_height;
	float fov = 60.0f;

	Vec3 camera_pos(0.0f, 0.0f, 0.0f);
	Vec3 camera_direction(0.0f, 0.0f, -1.0f);
	Vec3 screen_center = camera_pos + Deg2Rad(fov) * camera_direction;
	Vec3 screen_top_left = screen_center + Vec3(-aspect, 1.0f, 0.0f);
	Vec3 screen_top_right = screen_center + Vec3(aspect, 1.0f, 0.0f);
	Vec3 screen_bottom_left = screen_center + Vec3(-aspect, -1.0f, 0.0f);

	for (uint32_t y = 0; y < data.window_height; ++y)
	{
		for (uint32_t x = 0; x < data.window_width; ++x)
		{
			const float u = (float)x * (1.0f / data.window_width);
			const float v = (float)y * (1.0f / data.window_height);
			const Vec3 pixel_pos = screen_top_left + u * (screen_top_right - screen_top_left) + v * (screen_bottom_left - screen_top_left);

			Ray ray = { camera_pos, Vec3Normalize(pixel_pos - camera_pos) };
			Vec4 final_color = TraceRay(ray);
			data.pixels[y * data.window_width + x] = Vec4ToUint(final_color);
		}
	}
}

int main(int argc, char* argv[])
{
	CreateWindowArgs window_args = {};
	window_args.width = 1280;
	window_args.height = 720;
	window_args.title = L"Graphics Advanced Masterclass";
	CreateWindow(window_args);

	DX12::DX12InitArgs dx12_args = {};
	dx12_args.hWnd = data.hWnd;
	dx12_args.width = data.window_width;
	dx12_args.height = data.window_height;
	DX12::Init(dx12_args);

	data.pixels.resize(data.window_width * data.window_height);
	data.materials.emplace_back(Vec4(0.5f, 0.35f, 0.25f, 1.0f));
	data.materials.emplace_back(Vec4(0.1f, 0.25f, 0.8f, 1.0f));
	data.planes.emplace_back(Vec3(0.0f, -1.0f, 0.0f), Vec3(0.0f, -1.0f, 0.0f), 0);
	data.spheres.emplace_back(Vec3(0.0f, 0.0f, -2.0f), 1.0f * 1.0f, 1);

	std::chrono::high_resolution_clock::time_point curr_time = std::chrono::high_resolution_clock::now(),
		last_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<float> delta_time = std::chrono::duration<float>(0.0f);

	while (!data.should_close)
	{
		curr_time = std::chrono::high_resolution_clock::now();
		delta_time = curr_time - last_time;

		PollEvents();
		Update(delta_time.count());
		Render();

		DX12::CopyToBackBuffer(data.pixels.data(), data.pixels.size() * sizeof(uint32_t));
		DX12::Present();

		last_time = curr_time;
	}

	DX12::Exit();

	DestroyWindow();

	return 0;
}