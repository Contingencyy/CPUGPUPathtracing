#include "Input.h"
#include "Window.h"

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

#include <unordered_map>

namespace Input
{

	struct Data
	{
		std::unordered_map<WPARAM, KeyCode> key_mappings
		{
			{ 0x57, KeyCode_W }, { 0x53, KeyCode_S }, { 0x41, KeyCode_A }, { 0x44, KeyCode_D },
			{ VK_SPACE, KeyCode_Space }, { VK_SHIFT, KeyCode_LeftShift },
			{ VK_LBUTTON, KeyCode_LeftMouse }, { VK_RBUTTON, KeyCode_RightMouse }
		};
		std::vector<bool> key_states = std::vector<bool>(KeyCode_NumKeys);
		Vec2 mouse_pos_curr = Vec2(0.0f), mouse_pos_prev = Vec2(0.0f);
		Vec2 mouse_move_rel = Vec2(0.0f);
	} static data;

	void UpdateKeyState(WPARAM wparam, bool pressed)
	{
		if (data.key_mappings.find(wparam) != data.key_mappings.end())
		{
			data.key_states[data.key_mappings.at(wparam)] = pressed;
		}
	}

	void UpdateMousePosition()
	{
		POINT mouse_pos;
		GetCursorPos(&mouse_pos);
		data.mouse_pos_prev = data.mouse_pos_curr;
		data.mouse_pos_curr = Vec2(mouse_pos.x, mouse_pos.y);

		if (Window::IsMouseCaptured())
		{
			Window::ResetMousePosition(data.mouse_pos_prev.x, data.mouse_pos_prev.y);
		}
		
		data.mouse_move_rel.x = data.mouse_pos_curr.x - data.mouse_pos_prev.x;
		data.mouse_move_rel.y = data.mouse_pos_curr.y - data.mouse_pos_prev.y;
	}

	bool IsKeyPressed(KeyCode key)
	{
		return data.key_states[key];
	}

	float GetInputAxis1D(KeyCode pos_axis, KeyCode neg_axis)
	{
		return (float)data.key_states[pos_axis] + (-(float)data.key_states[neg_axis]);
	}

	Vec2 GetMouseMoveRel()
	{
		return data.mouse_move_rel;
	}

};
