#pragma once
#include "MathLib.h"

struct Vertex;

#include <vector>
#include <string>

namespace GLTFLoader
{

	struct Mesh
	{
		std::vector<Vertex> vertices;
		std::vector<uint32_t> indices;
	};

	GLTFLoader::Mesh Load(const std::string& filepath);

}
