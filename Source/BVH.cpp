#include "BVH.h"

#include <algorithm>

void BVH::Build(const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices, BVHBuildOption build_option)
{
	m_build_option = build_option;

	m_triangles.resize(indices.size() / 3);
	for (uint32_t i = 0, curr_index = 0; i < m_triangles.size(); ++i, curr_index += 3)
	{
		m_triangles[i].v0 = vertices[indices[curr_index]].pos;
		m_triangles[i].v1 = vertices[indices[curr_index + 1]].pos;
		m_triangles[i].v2 = vertices[indices[curr_index + 2]].pos;
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

	m_cheapest_tri_indices.resize(m_tri_indices.size());
	m_curr_tri_indices.resize(m_tri_indices.size());

	m_nodes.resize(m_triangles.size() * 2 - 1);

	BVHNode& root_node = m_nodes[m_current_node++];
	root_node.left_first = 0;
	root_node.prim_count = (uint32_t)m_triangles.size();

	CalculateNodeBounds(root_node, m_tri_indices);
	Subdivide(0, 0);
}

void BVH::Rebuild(BVHBuildOption build_option)
{
	m_build_option = build_option;
	m_current_node = 0;

	BVHNode& root_node = m_nodes[m_current_node++];
	root_node.left_first = 0;
	root_node.prim_count = (uint32_t)m_triangles.size();

	CalculateNodeBounds(root_node, m_tri_indices);
	Subdivide(0, 0);
}

void BVH::Traverse(Ray& ray)
{
	Intersect(ray, 0);
}

Triangle BVH::GetTriangle(uint32_t index) const
{
	return m_triangles[m_tri_indices[index]];
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

	if (m_build_option == BVHBuildOption_NaiveSplit)
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
	else if (m_build_option == BVHBuildOption_SAHSplit)
	{
		// Surface area heuristic
		// Go through each primitive, and for each axis, run the split plane through their centroid
		// If the split is less expensive than the previous one, update the split values kept outside of the loop
		// Note: The cost of a split is determined by the area of the left and right split box plus the number of primitives in each
		// We do not try to find the most optimal splits across all levels of the BVH, we simply assume that the current level is the final split (greedy)
		BVHNode& node = m_nodes[node_index];
		float parent_node_cost = GetAABBVolume(node.bounds) * node.prim_count;

		bool cheaper_split_found = false;
		float cheapest_split_cost = 1e34f;
		BVHNode cheapest_left_node = {};
		BVHNode cheapest_right_node = {};

		BVHNode current_left_node = {};
		BVHNode current_right_node = {};

		for (uint32_t tri_idx = node.left_first; tri_idx < node.left_first + node.prim_count; ++tri_idx)
		{
			const Triangle& tri = m_triangles[m_tri_indices[tri_idx]];
			const Vec3& centroid = m_centroids[m_tri_indices[tri_idx]];

			for (uint32_t curr_axis = 0; curr_axis < 3; ++curr_axis)
			{
				m_curr_tri_indices = m_tri_indices;

				float split_pos = centroid.xyz[curr_axis];
				int32_t i = node.left_first;
				int32_t j = i + node.prim_count - 1;

				while (i <= j)
				{
					if (m_centroids[m_curr_tri_indices[i]].xyz[curr_axis] < split_pos)
					{
						i++;
					}
					else
					{
						std::swap(m_curr_tri_indices[i], m_curr_tri_indices[j--]);
					}
				}

				uint32_t left_count = i - node.left_first;
				if (left_count == 0 || left_count == node.prim_count)
					continue;

				current_left_node.left_first = node.left_first;
				current_left_node.prim_count = left_count;
				current_right_node.left_first = i;
				current_right_node.prim_count = node.prim_count - left_count;

				CalculateNodeBounds(current_left_node, m_curr_tri_indices);
				CalculateNodeBounds(current_right_node, m_curr_tri_indices);

				float split_cost = GetAABBVolume(current_left_node.bounds) * current_left_node.prim_count +
					GetAABBVolume(current_right_node.bounds) * current_right_node.prim_count;

				// We update the cheapest split if the split is cheaper than the parent cost
				if (split_cost < parent_node_cost && split_cost < cheapest_split_cost)
				{
					cheaper_split_found = true;
					cheapest_split_cost = split_cost;

					cheapest_left_node = current_left_node;
					cheapest_right_node = current_right_node;
					m_cheapest_tri_indices = m_curr_tri_indices;
				}
			}
		}

		// We can terminate the splitting, no cheaper split was found
		if (!cheaper_split_found)
		{
			return;
		}

		uint32_t left_child_index = m_current_node++;
		uint32_t right_child_index = m_current_node++;

		m_nodes[left_child_index] = cheapest_left_node;
		m_nodes[right_child_index] = cheapest_right_node;
		m_tri_indices = m_cheapest_tri_indices;

		node.left_first = left_child_index;
		node.prim_count = 0;

		Subdivide(left_child_index, depth + 1);
		Subdivide(right_child_index, depth + 1);
	}
}

void BVH::Intersect(Ray& ray, uint32_t node_index)
{
	BVHNode& node = m_nodes[node_index];

	if (!IntersectAABB(node.bounds, ray))
	{
		return;
	}

	ray.payload.bvh_depth += 1;

	if (node.prim_count > 0)
	{
		for (uint32_t i = 0; i < node.prim_count; ++i)
		{
			const Triangle& tri = m_triangles[m_tri_indices[node.left_first + i]];
			bool intersected = IntersectTriangle(tri, ray);

			if (intersected)
			{
				ray.payload.tri_idx = m_tri_indices[node.left_first + i];
			}
		}
	}
	else
	{
		Intersect(ray, node.left_first);
		Intersect(ray, node.left_first + 1);
	}
}
