#pragma once
#include "MathLib.h"

struct Vertex;
struct Mesh;

namespace GLTFLoader
{

	Mesh Load(const std::string& filepath);

}
