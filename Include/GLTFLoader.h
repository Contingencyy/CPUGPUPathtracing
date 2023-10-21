#pragma once
#include "MathLib.h"

struct Vertex;
struct Mesh;

#include <vector>
#include <string>

namespace GLTFLoader
{

	Mesh Load(const std::string& filepath);

}
