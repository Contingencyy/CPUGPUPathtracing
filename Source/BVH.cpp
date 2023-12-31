#include "Common.h"
#include "BVH.h"

#include "imgui/imgui.h"

const std::vector<const char*> bvh_build_option_names =
{
	"Naive split", "SAH split intervals", "SAH split primitives"
};

void BVH::Build(const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices, BuildOption build_option)
{
	m_current_build_option = build_option;

	m_triangles.resize(indices.size() / 3);
	for (uint32_t i = 0, curr_index = 0; i < m_triangles.size(); ++i, curr_index += 3)
	{
		m_triangles[i].v0 = vertices[indices[curr_index]];
		m_triangles[i].v1 = vertices[indices[curr_index + 1]];
		m_triangles[i].v2 = vertices[indices[curr_index + 2]];

		m_total_area += GetTriangleArea(m_triangles[i]);
	}

	m_tri_indices.resize(m_triangles.size());
	for (uint32_t i = 0; i < m_tri_indices.size(); ++i)
	{
		m_tri_indices[i] = i;
	}

	m_centroids.resize(m_triangles.size());
	for (uint32_t i = 0; i < m_triangles.size(); ++i)
	{
		m_centroids[i] = TriangleCentroid(m_triangles[i]);
	}

	m_nodes.resize(m_triangles.size() * 2 - 1);

	BVHNode& root_node = m_nodes[m_current_node++];
	root_node.left_first = 0;
	root_node.prim_count = (uint32_t)m_triangles.size();

	CalculateNodeBounds(root_node, m_tri_indices);
	Subdivide(0, 0);
}

void BVH::Rebuild(BuildOption build_option)
{
	m_current_build_option = build_option;
	m_current_node = 0;
	m_max_depth = 0;

	BVHNode& root_node = m_nodes[m_current_node++];
	root_node.left_first = 0;
	root_node.prim_count = (uint32_t)m_triangles.size();

	CalculateNodeBounds(root_node, m_tri_indices);
	Subdivide(0, 0);
}

bool BVH::Traverse(Ray& ray) const
{
	bool result = false;

	const BVHNode* node = &m_nodes[0];
	const BVHNode* stack[64] = {};
	uint32_t stack_ptr = 0;

	while (true)
	{
		// The current node is a leaf node, we need to test intersections with its primitives
		if (node->prim_count > 0)
		{
			for (uint32_t tri_idx = node->left_first; tri_idx < node->left_first + node->prim_count; ++tri_idx)
			{
				const Triangle& tri = m_triangles[m_tri_indices[tri_idx]];
				bool intersected = IntersectTriangle(tri, ray);

				if (intersected)
				{
					ray.payload.tri_idx = m_tri_indices[tri_idx];
					result = true;
				}
			}

			if (stack_ptr == 0)
				break;
			else
				node = stack[--stack_ptr];
			continue;
		}

		const BVHNode* left_child = &m_nodes[node->left_first];
		const BVHNode* right_child = &m_nodes[node->left_first + 1];

		// Get the distance to both the left and right child nodes
		float left_dist = IntersectAABB_SSE(left_child->aabb_min4, left_child->aabb_max4, ray);
		float right_dist = IntersectAABB_SSE(right_child->aabb_min4, right_child->aabb_max4, ray);

		// Swap the left and right children so that we now have the closest in the left child
		if (left_dist > right_dist)
		{
			std::swap(left_dist, right_dist);
			std::swap(left_child, right_child);
		}

		// If the closest distance is invalid, we have not hit any boxes, so we can continue traversing the stack
		if (left_dist == 1e30f)
		{
			if (stack_ptr == 0)
				break;
			else
				node = stack[--stack_ptr];
		}
		else
		{
			// We have intersected with a child node, now check the closest node first and push the other one to the stack
			ray.payload.bvh_depth++;

			node = left_child;
			if (right_dist != 1e30f)
				stack[stack_ptr++] = right_child;
		}
	}

	return result;
}

Triangle BVH::GetTriangle(uint32_t index) const
{
	return m_triangles[index];
}

uint32_t BVH::NumTriangles() const
{
	return (uint32_t)m_triangles.size();
}

uint32_t BVH::GetMaxDepth() const
{
	return m_max_depth;
}

float BVH::GetTotalArea() const
{
	return m_total_area;
}

void BVH::RenderImGui()
{
	ImGui::Text("Triangle count: %u", m_triangles.size());
	ImGui::Text("BVH Depth: %u", m_max_depth);
	ImGui::Text("BVH Area: %.3f", m_total_area);

	bool is_built = m_selected_build_option == m_current_build_option;
	if (!is_built)
		ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0.6f, 0.0f, 0.0f, 1.0f));

	if (ImGui::BeginCombo("Build options", bvh_build_option_names[m_selected_build_option]))
	{
		for (size_t i = 0; i < BVH::BuildOption_NumOptions; ++i)
		{
			bool is_selected = i == m_selected_build_option;

			if (ImGui::Selectable(bvh_build_option_names[i], is_selected))
			{
				m_selected_build_option = (BVH::BuildOption)i;
			}

			if (is_selected)
			{
				ImGui::SetItemDefaultFocus();
			}
		}

		ImGui::EndCombo();
	}

	if (!is_built)
		ImGui::PopStyleColor();

	if (ImGui::Button("Rebuild BVH"))
	{
		Rebuild(m_selected_build_option);
	}
}

void BVH::CalculateNodeBounds(BVHNode& node, const std::vector<uint32_t>& tri_indices)
{
	node.aabb_min = Vec3(1e30f);
	node.aabb_max = Vec3(-1e30f);

	for (uint32_t i = node.left_first; i < node.left_first + node.prim_count; ++i)
	{
		uint32_t tri_idx = tri_indices[i];
		const Triangle& tri = m_triangles[tri_idx];

		AABB tri_bounds = TriangleBounds(tri);
		node.aabb_min = Vec3Min(node.aabb_min, tri_bounds.pmin);
		node.aabb_max = Vec3Max(node.aabb_max, tri_bounds.pmax);
	}
}

void BVH::Subdivide(uint32_t node_index, uint32_t depth)
{
	m_max_depth = std::max(m_max_depth, depth);

	if (m_current_build_option == BuildOption_NaiveSplit)
	{
		BVHNode& node = m_nodes[node_index];
		if (node.prim_count <= 2)
			return;

		Vec3 extent = node.aabb_max - node.aabb_min;
		uint32_t axis = 0;

		if (extent.y > extent.x)
			axis = 1;
		if (extent.z > extent.xyz[axis])
			axis = 2;

		float split_pos = node.aabb_min.xyz[axis] + extent.xyz[axis] * 0.5f;
		Split(node, axis, split_pos, depth);
	}
	else if (m_current_build_option == BuildOption_SAHSplitIntervals)
	{
		BVHNode& node = m_nodes[node_index];
		float parent_cost = GetAABBVolume(node.aabb_min, node.aabb_max) * node.prim_count;

		float cheapest_cost = 1e30f;
		uint32_t cheapest_split_axis = 0;
		float cheapest_split_pos = 0.0f;

		for (uint32_t split_idx = 0; split_idx < 8; ++split_idx)
		{
			for (uint32_t curr_axis = 0; curr_axis < 3; ++curr_axis)
			{
				float axis_width = node.aabb_max.xyz[curr_axis] - node.aabb_min.xyz[curr_axis];
				float split_pos = axis_width * ((float)split_idx / 8) + node.aabb_min.xyz[curr_axis];
				float split_cost = EvaluateSAH(node, curr_axis, split_pos);

				// We update the cheapest split if the split is cheaper than the parent cost
				if (split_cost < cheapest_cost)
				{
					cheapest_cost = split_cost;
					cheapest_split_axis = curr_axis;
					cheapest_split_pos = split_pos;
				}
			}
		}

		// We can terminate the splitting, no cheaper split was found
		if (cheapest_cost >= parent_cost)
		{
			return;
		}

		Split(node, cheapest_split_axis, cheapest_split_pos, depth);
	}
	else if (m_current_build_option == BuildOption_SAHSplitPrimitives)
	{
		BVHNode& node = m_nodes[node_index];
		float parent_cost = GetAABBVolume(node.aabb_min, node.aabb_max) * node.prim_count;

		float cheapest_cost = 1e30f;
		uint32_t cheapest_split_axis = 0;
		float cheapest_split_pos = 0.0f;

		for (uint32_t tri_idx = node.left_first; tri_idx < node.left_first + node.prim_count; ++tri_idx)
		{
			const Triangle& tri = m_triangles[m_tri_indices[tri_idx]];
			const Vec3& centroid = m_centroids[m_tri_indices[tri_idx]];

			for (uint32_t curr_axis = 0; curr_axis < 3; ++curr_axis)
			{
				float axis_width = node.aabb_max.xyz[curr_axis] - node.aabb_min.xyz[curr_axis];
				float split_pos = centroid.xyz[curr_axis];
				float split_cost = EvaluateSAH(node, curr_axis, split_pos);

				// We update the cheapest split if the split is cheaper than the parent cost
				if (split_cost < cheapest_cost)
				{
					cheapest_split_axis = curr_axis;
					cheapest_split_pos = split_pos;
				}
			}
		}

		// We can terminate the splitting, no cheaper split was found
		if (cheapest_cost >= parent_cost)
		{
			return;
		}

		Split(node, cheapest_split_axis, cheapest_split_pos, depth);
	}
}

float BVH::EvaluateSAH(BVHNode& node, uint32_t axis, float split_pos)
{
	AABB left_bounds = { Vec3(1e30f), Vec3(-1e30f) }, right_bounds = { Vec3(1e30f), Vec3(-1e30f) };
	uint32_t left_count = 0, right_count = 0;

	for (uint32_t tri_idx = node.left_first; tri_idx < node.left_first + node.prim_count; ++tri_idx)
	{
		const Triangle& tri = m_triangles[m_tri_indices[tri_idx]];
		const Vec3& centroid = m_centroids[m_tri_indices[tri_idx]];

		if (centroid.xyz[axis] < split_pos)
		{
			left_count++;
			GrowAABB(left_bounds.pmin, left_bounds.pmax, tri.v0.pos);
			GrowAABB(left_bounds.pmin, left_bounds.pmax, tri.v1.pos);
			GrowAABB(left_bounds.pmin, left_bounds.pmax, tri.v2.pos);
		}
		else
		{
			right_count++;
			GrowAABB(right_bounds.pmin, right_bounds.pmax, tri.v0.pos);
			GrowAABB(right_bounds.pmin, right_bounds.pmax, tri.v1.pos);
			GrowAABB(right_bounds.pmin, right_bounds.pmax, tri.v2.pos);
		}
	}

	float split_cost = left_count * GetAABBVolume(left_bounds.pmin, left_bounds.pmax) + right_count * GetAABBVolume(right_bounds.pmin, right_bounds.pmax);
	return split_cost;
}

void BVH::Split(BVHNode& node, uint32_t axis, float split_pos, uint32_t depth)
{
	int32_t i = node.left_first;
	int32_t j = i + node.prim_count - 1;

	while (i <= j)
	{
		if (m_centroids[m_tri_indices[i]].xyz[axis] < split_pos)
		{
			i++;
		}
		else
		{
			std::swap(m_tri_indices[i], m_tri_indices[j--]);
		}
	}

	uint32_t left_count = i - node.left_first;
	if (left_count == 0 || left_count == node.prim_count)
		return;

	uint32_t left_child_index = m_current_node++;
	uint32_t right_child_index = m_current_node++;

	m_nodes[left_child_index].left_first = node.left_first;
	m_nodes[left_child_index].prim_count = left_count;
	m_nodes[right_child_index].left_first = i;
	m_nodes[right_child_index].prim_count = node.prim_count - left_count;

	node.left_first = left_child_index;
	node.prim_count = 0;

	CalculateNodeBounds(m_nodes[left_child_index], m_tri_indices);
	CalculateNodeBounds(m_nodes[right_child_index], m_tri_indices);

	Subdivide(left_child_index, depth + 1);
	Subdivide(right_child_index, depth + 1);
}
