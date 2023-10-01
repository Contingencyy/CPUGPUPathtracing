#include "Window.h"
#include "Input.h"

#include "imgui/imgui_impl_win32.h"

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

#define LOWORD(l) ((WORD)(((DWORD_PTR)(l)) & 0xffff))
#define HIWORD(l) ((WORD)((((DWORD_PTR)(l)) >> 16) & 0xffff))
#define GET_X_LPARAM(lp) ((int)(short)LOWORD(lp))
#define GET_Y_LPARAM(lp) ((int)(short)HIWORD(lp))

extern LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
LRESULT CALLBACK WindowEventCallback(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	if (ImGui_ImplWin32_WndProcHandler(hWnd, message, wParam, lParam))
	{
		return true;
	}

	switch (message)
	{
	case WM_SYSKEYDOWN:
	case WM_KEYDOWN:
	case WM_LBUTTONDOWN:
	case WM_RBUTTONDOWN:
	{
		Input::UpdateKeyState(wParam, true);
	} break;
	case WM_SYSKEYUP:
	case WM_KEYUP:
	{
		Input::UpdateKeyState(wParam, false);
	} break;
	case WM_LBUTTONUP:
	{
		Input::UpdateKeyState(VK_LBUTTON, false);
	} break;
	case WM_RBUTTONUP:
	{
		Input::UpdateKeyState(VK_RBUTTON, false);
	} break;

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

namespace Window
{

	struct Data
	{
		HWND hWnd = nullptr;
		uint32_t window_width = 1280;
		uint32_t window_height = 720;
		bool mouse_capture = false;
		bool should_close = false;
	} static data;

	void Create(const CreateWindowArgs& args)
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

	void Destroy()
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

	bool ShouldClose()
	{
		return data.should_close;
	}

	void SetMouseCapture(bool capture)
	{
		if (data.mouse_capture != capture)
		{
			data.mouse_capture = capture;

			RECT window_rect;
			::GetWindowRect(data.hWnd, &window_rect);
			ShowCursor(!capture);
			ClipCursor(capture ? &window_rect : nullptr);
		}
	}

	bool IsMouseCaptured()
	{
		return data.mouse_capture;
	}

	void ResetMousePosition(float& x, float& y)
	{
		RECT window_rect;
		::GetWindowRect(data.hWnd, &window_rect);

		uint32_t window_width = window_rect.right - window_rect.left;
		uint32_t window_height = window_rect.bottom - window_rect.top;

		uint32_t center_x = window_rect.left + window_width / 2;
		uint32_t center_y = window_rect.top + window_height / 2;

		SetCursorPos(center_x, center_y);
		x = (float)center_x;
		y = (float)center_y;
	}

	HWND GetHandle()
	{
		return data.hWnd;
	}

	UVec2 GetFramebufferSize()
	{
		return UVec2(data.window_width, data.window_height);
	}

}
