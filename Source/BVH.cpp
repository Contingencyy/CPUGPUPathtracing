#include "BVH.h"

#include <algorithm>

void BVH::Build(const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices, BuildOption build_option)
{
	m_build_option = build_option;

	m_triangles.resize(indices.size() / 3);
	for (uint32_t i = 0, curr_index = 0; i < m_triangles.size(); ++i, curr_index += 3)
	{
		m_triangles[i].v0 = vertices[indices[curr_index]];
		m_triangles[i].v1 = vertices[indices[curr_index + 1]];
		m_triangles[i].v2 = vertices[indices[curr_index + 2]];
	}

	m_tri_indices.resize(m_triangles.size());
	for (uint32_t i = 0; i < m_tri_indices.size(); ++i)
	{
		m_tri_indices[i] = i;
	}

	m_centroids.resize(m_triangles.size());
	for (uint32_t i = 0; i < m_triangles.size(); ++i)
	{
		m_centroids[i] = GetTriangleCentroid(m_triangles[i]);
	}

	m_nodes.resize(m_triangles.size() * 2 - 1);

	BVHNode& root_node = m_nodes[m_current_node++];
	root_node.left_first = 0;
	root_node.prim_count = (uint32_t)m_triangles.size();

	CalculateNodeBounds(root_node, m_tri_indices);
	Subdivide(0, 0);
}

void BVH::Rebuild()
{
	m_current_node = 0;

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
		float left_dist = IntersectAABB(left_child->bounds, ray);
		float right_dist = IntersectAABB(right_child->bounds, ray);

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

uint32_t BVH::GetMaxDepth() const
{
	return m_max_depth;
}

void BVH::CalculateNodeBounds(BVHNode& node, const std::vector<uint32_t>& tri_indices)
{
	node.bounds = { Vec3(1e30f), Vec3(-1e30f) };

	for (uint32_t i = node.left_first; i < node.left_first + node.prim_count; ++i)
	{
		uint32_t tri_idx = tri_indices[i];
		const Triangle& tri = m_triangles[tri_idx];

		AABB tri_bounds = GetTriangleBounds(tri);
		node.bounds.pmin = Vec3Min(node.bounds.pmin, tri_bounds.pmin);
		node.bounds.pmax = Vec3Max(node.bounds.pmax, tri_bounds.pmax);
	}
}

void BVH::Subdivide(uint32_t node_index, uint32_t depth)
{
	m_max_depth = std::max(m_max_depth, depth);

	if (m_build_option == BuildOption_NaiveSplit)
	{
		BVHNode& node = m_nodes[node_index];
		if (node.prim_count <= 2)
			return;

		Vec3 extent = node.bounds.pmax - node.bounds.pmin;
		uint32_t axis = 0;

		if (extent.y > extent.x)
			axis = 1;
		if (extent.z > extent.xyz[axis])
			axis = 2;

		float split_pos = node.bounds.pmin.xyz[axis] + extent.xyz[axis] * 0.5f;
		Split(node, axis, split_pos, depth);
	}
	else if (m_build_option == BuildOption_SAHSplitIntervals)
	{
		BVHNode& node = m_nodes[node_index];
		float parent_cost = GetAABBVolume(node.bounds) * node.prim_count;

		float cheapest_cost = 1e30f;
		uint32_t cheapest_split_axis = 0;
		float cheapest_split_pos = 0.0f;

		for (uint32_t split_idx = 0; split_idx < 8; ++split_idx)
		{
			for (uint32_t curr_axis = 0; curr_axis < 3; ++curr_axis)
			{
				float axis_width = node.bounds.pmax.xyz[curr_axis] - node.bounds.pmin.xyz[curr_axis];
				float split_pos = axis_width * ((float)split_idx / 8) + node.bounds.pmin.xyz[curr_axis];
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
	else if (m_build_option == BuildOption_SAHSplitPrimitives)
	{
		BVHNode& node = m_nodes[node_index];
		float parent_cost = GetAABBVolume(node.bounds) * node.prim_count;

		float cheapest_cost = 1e30f;
		uint32_t cheapest_split_axis = 0;
		float cheapest_split_pos = 0.0f;

		for (uint32_t tri_idx = node.left_first; tri_idx < node.left_first + node.prim_count; ++tri_idx)
		{
			const Triangle& tri = m_triangles[m_tri_indices[tri_idx]];
			const Vec3& centroid = m_centroids[m_tri_indices[tri_idx]];

			for (uint32_t curr_axis = 0; curr_axis < 3; ++curr_axis)
			{
				float axis_width = node.bounds.pmax.xyz[curr_axis] - node.bounds.pmin.xyz[curr_axis];
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
			GrowAABB(left_bounds, tri.v0.pos);
			GrowAABB(left_bounds, tri.v1.pos);
			GrowAABB(left_bounds, tri.v2.pos);
		}
		else
		{
			right_count++;
			GrowAABB(right_bounds, tri.v0.pos);
			GrowAABB(right_bounds, tri.v1.pos);
			GrowAABB(right_bounds, tri.v2.pos);
		}
	}

	float split_cost = left_count * GetAABBVolume(left_bounds) + right_count * GetAABBVolume(right_bounds);
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
