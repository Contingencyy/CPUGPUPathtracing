#include "DX12.h"

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
	RECT window_rect = {};
	RECT client_rect = {};

	// Application data
	bool should_close = false;
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

	data.window_rect = { 0, 0, static_cast<LONG>(args.width), static_cast<LONG>(args.height) };
	::AdjustWindowRect(&data.window_rect, WS_OVERLAPPEDWINDOW, FALSE);

	int window_width = data.window_rect.right - data.window_rect.left;
	int window_height = data.window_rect.bottom - data.window_rect.top;

	int window_x = std::max(0, (screen_width - window_width) / 2);
	int window_y = std::max(0, (screen_height - window_height) / 2);

	data.hWnd = ::CreateWindowExW(NULL, windowClassName, args.title, WS_OVERLAPPEDWINDOW,
		window_x, window_y, window_width, window_height, NULL, NULL, hInst, nullptr);
	
	if (!data.hWnd)
	{
		throw std::runtime_error("Failed to create window");
	}

	::GetClientRect(data.hWnd, &data.client_rect);
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

void Render()
{
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
	dx12_args.width = data.client_rect.right - data.client_rect.left;
	dx12_args.height = data.client_rect.bottom - data.client_rect.top;
	DX12::Init(dx12_args);

	std::chrono::high_resolution_clock::time_point curr_time = std::chrono::high_resolution_clock::now(),
		last_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<float> delta_time = std::chrono::duration<float>(0.0f);

	size_t pixels_byte_size = dx12_args.width * dx12_args.height * 4;
	uint32_t* test_pixels = (uint32_t*)malloc(pixels_byte_size);
	memset(test_pixels, 128, pixels_byte_size);

	while (!data.should_close)
	{
		curr_time = std::chrono::high_resolution_clock::now();
		delta_time = curr_time - last_time;

		PollEvents();
		Update(delta_time.count());
		Render();

		DX12::CopyToBackBuffer(test_pixels, pixels_byte_size);
		DX12::Present();

		last_time = curr_time;
	}

	free(test_pixels);

	DX12::Exit();

	DestroyWindow();

	return 0;
}