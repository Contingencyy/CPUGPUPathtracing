#pragma once
#include "MathLib.h"

class HWND__;
typedef HWND__* HWND;

namespace Window
{

	struct CreateWindowArgs
	{
		uint32_t width = 1280;
		uint32_t height = 720;
		const wchar_t* title = L"DefaultWindowTitle";
	};

	void Create(const CreateWindowArgs& args);
	void Destroy();

	void PollEvents();
	bool ShouldClose();
	void SetMouseCapture(bool capture);
	bool IsMouseCaptured();
	void ResetMousePosition(float& x, float& y);

	HWND GetHandle();
	UVec2 GetFramebufferSize();

}
