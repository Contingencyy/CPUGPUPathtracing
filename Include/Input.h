#pragma once
#include "MathLib.h"

#include <stdint.h>

typedef unsigned __int64 UINT_PTR;
typedef UINT_PTR WPARAM;

namespace Input
{

	enum KeyCode : uint16_t
	{
		KeyCode_W, KeyCode_S, KeyCode_A, KeyCode_D,
		KeyCode_Space, KeyCode_LeftShift,
		KeyCode_LeftMouse, KeyCode_RightMouse,
		KeyCode_NumKeys
	};

	void UpdateKeyState(WPARAM wparam, bool pressed);
	void UpdateMousePosition();

	bool IsKeyPressed(KeyCode key);
	float GetInputAxis1D(KeyCode pos_axis, KeyCode neg_axis);
	Vec2 GetMouseMoveRel();

}
