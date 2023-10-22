#pragma once

class HWND__;
typedef HWND__* HWND;

namespace DX12
{

	struct DX12InitArgs
	{
		HWND hWnd = nullptr;
		uint32_t width = 0;
		uint32_t height = 0;
	};

	void Init(const DX12InitArgs& args);
	void Exit();

	void CopyToBackBuffer(uint32_t* pixels, size_t num_bytes);
	void Present();

}
