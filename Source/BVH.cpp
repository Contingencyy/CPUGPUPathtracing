#include "BVH.h"

#include <algorithm>

void BVH::Build(const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices)
{
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

	m_nodes.resize(m_triangles.size() * 2 - 1);

	BVHNode& root_node = m_nodes[m_current_node++];
	root_node.left_first = 0;
	root_node.prim_count = (uint32_t)m_triangles.size();

	CalculateNodeBounds(0);
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

void BVH::CalculateNodeBounds(uint32_t node_index)
{
	BVHNode& node = m_nodes[node_index];
	node.bounds = { Vec3(1e30f), Vec3(-1e30f) };

	for (uint32_t i = node.left_first; i < node.left_first + node.prim_count; ++i)
	{
		uint32_t tri_index = m_tri_indices[i];
		const Triangle& tri = m_triangles[tri_index];

		AABB tri_bounds = GetTriangleBounds(tri);
		node.bounds.pmin = Vec3Min(node.bounds.pmin, tri_bounds.pmin);
		node.bounds.pmax = Vec3Max(node.bounds.pmax, tri_bounds.pmax);
	}
}

void BVH::Subdivide(uint32_t node_index, uint32_t depth)
{
	m_max_depth = std::max(m_max_depth, depth);

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

	CalculateNodeBounds(left_child_index);
	CalculateNodeBounds(right_child_index);

	Subdivide(left_child_index, depth + 1);
	Subdivide(right_child_index, depth + 1);
}

void BVH::Intersect(Ray& ray, uint32_t node_index)
{
	BVHNode& node = m_nodes[node_index];

	if (!IntersectAABB(node.bounds, ray))
	{
		return;
	}

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
		ray.payload.bvh_depth += 1;

		Intersect(ray, node.left_first);
		Intersect(ray, node.left_first + 1);
	}
}
