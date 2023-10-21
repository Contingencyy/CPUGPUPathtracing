#include "GLTFLoader.h"
#include "Primitives.h"

#define CGLTF_IMPLEMENTATION
#include "cgltf/cgltf.h"

#include <stdexcept>

template<typename T>
static T* CGLTFGetDataPointer(const cgltf_accessor* accessor)
{
	cgltf_buffer_view* buffer_view = accessor->buffer_view;
	uint8_t* base_ptr = (uint8_t*)(buffer_view->buffer->data);
	base_ptr += buffer_view->offset;
	base_ptr += accessor->offset;

	return (T*)base_ptr;
}

Mesh GLTFLoader::Load(const std::string& filepath)
{
	Mesh mesh;

	cgltf_options options = {};
	cgltf_data* data = nullptr;
	cgltf_result parsed = cgltf_parse_file(&options, filepath.c_str(), &data);

	if (parsed != cgltf_result_success)
	{
		throw std::runtime_error("Could not load GLTF");
	}

	cgltf_load_buffers(&options, data, filepath.c_str());

	for (size_t mesh_idx = 0; mesh_idx < data->meshes_count; ++mesh_idx)
	{
		const cgltf_mesh& gltf_mesh = data->meshes[mesh_idx];

		for (size_t prim_idx = 0; prim_idx < gltf_mesh.primitives_count; ++prim_idx)
		{
			const cgltf_primitive& gltf_prim = gltf_mesh.primitives[prim_idx];
			mesh.indices.resize(gltf_prim.indices->count);
			mesh.vertices.resize(gltf_prim.attributes[0].data->count);

			for (size_t attr_idx = 0; attr_idx < gltf_prim.attributes_count; ++attr_idx)
			{
				const cgltf_attribute& gltf_attr = gltf_prim.attributes[attr_idx];

				if (gltf_prim.indices->component_type == cgltf_component_type_r_32u)
				{
					memcpy(mesh.indices.data(), CGLTFGetDataPointer<uint32_t>(gltf_prim.indices), sizeof(uint32_t) * gltf_prim.indices->count);
				}
				else if (gltf_prim.indices->component_type == cgltf_component_type_r_16u)
				{
					uint16_t* indices_ptr = CGLTFGetDataPointer<uint16_t>(gltf_prim.indices);

					for (size_t k = 0; k < gltf_prim.indices->count; ++k)
					{
						mesh.indices[k] = indices_ptr[k];
					}
				}

				switch (gltf_attr.type)
				{
				case cgltf_attribute_type_position:
				{
					Vec3* pos_ptr = CGLTFGetDataPointer<Vec3>(gltf_attr.data);

					for (size_t vert_idx = 0; vert_idx < gltf_attr.data->count; ++vert_idx)
					{
						mesh.vertices[vert_idx].pos = pos_ptr[vert_idx];
					}
				} break;
				case cgltf_attribute_type_normal:
				{
					Vec3* normal_ptr = CGLTFGetDataPointer<Vec3>(gltf_attr.data);

					for (size_t vert_idx = 0; vert_idx < gltf_attr.data->count; ++vert_idx)
					{
						mesh.vertices[vert_idx].normal = normal_ptr[vert_idx];
					}
				} break;
				}
			}
		}
	}

	cgltf_free(data);
	return mesh;
}
