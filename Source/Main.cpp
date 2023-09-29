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

struct Data
{
	// Window data
	HWND hWnd = nullptr;
	uint32_t window_width = 1280;
	uint32_t window_height = 720;

	// Application data
	bool should_close = false;

	// Render data
	size_t num_pixels = 0;
	uint32_t* pixels = nullptr;
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

struct Ray
{
	Vec3 origin = Vec3(0.0f);
	Vec3 direction = Vec3(0.0f);
	float t = 1e34f;

	struct Payload
	{
		Vec4 color = Vec4(0.0f);
	} payload;
};

struct Plane
{
	Vec3 normal = Vec3(0.0f);
	Vec3 point = Vec3(0.0f);
};

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
			ray.payload.color = Vec4(1.0f, 1.0f, 0.0f, 1.0f);
		}
	}
}

struct Sphere
{
	Vec3 center = Vec3(0.0f);
	float radius_sq = 0.0f;
};

void IntersectSphere(const Sphere& sphere, Ray& ray)
{
	float t0, t1;
#if 0
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
	ray.payload.color = Vec4(1.0f, 0.0f, 1.0f, 1.0f);
}

void TraceRay(Ray& ray)
{
	Plane plane = { Vec3Normalize(Vec3(5.0f, 2.0f, 1.0f)), Vec3(0.0f, 0.0f, -5.0f)};
	//IntersectPlane(plane, ray);

	Sphere sphere = { Vec3(0.0f, 0.0f, -1.5f), 1.0f * 1.0f };
	IntersectSphere(sphere, ray);
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
			TraceRay(ray);
			data.pixels[y * data.window_width + x] = Vec4ToUint(ray.payload.color);
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

	data.num_pixels = data.window_width * data.window_height;
	data.pixels = (uint32_t*)malloc(data.num_pixels * sizeof(uint32_t));
	memset(data.pixels, 0, data.num_pixels * sizeof(uint32_t));

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

		DX12::CopyToBackBuffer(data.pixels, data.num_pixels * sizeof(uint32_t));
		DX12::Present();

		last_time = curr_time;
	}

	free(data.pixels);

	DX12::Exit();

	DestroyWindow();

	return 0;
}