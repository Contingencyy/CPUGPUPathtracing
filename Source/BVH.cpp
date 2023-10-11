#include "BVH.h"

#include <algorithm>

static inline Vec3 CalculateTriangleCentroid(const Triangle& triangle)
{
	return (triangle.v0 + triangle.v1 + triangle.v2) * 0.3333f;
}

void BVH::Build(const std::vector<Triangle>& triangles)
{
	m_triangles = triangles;
	for (uint32_t i = 0; i < m_triangles.size(); ++i)
	{
		m_triangles[i].centroid = CalculateTriangleCentroid(m_triangles[i]);
	}

	m_tri_indices.resize(m_triangles.size());
	for (uint32_t i = 0; i < m_tri_indices.size(); ++i)
	{
		m_tri_indices[i] = i;
	}

	m_nodes.resize(m_triangles.size() * 2 - 1);

	BVHNode& root_node = m_nodes[m_current_node++];
	root_node.left_first = 0;
	root_node.prim_count = (uint32_t)m_triangles.size();

	CalculateNodeBounds(0);
	Subdivide(0);
}

void BVH::Traverse(Ray& ray)
{
	Intersect(ray, 0);
}

void BVH::CalculateNodeBounds(uint32_t node_index)
{
	BVHNode& node = m_nodes[node_index];
	node.bounds = { Vec3(1e30f), Vec3(-1e30f) };

	for (uint32_t i = node.left_first; i < node.left_first + node.prim_count; ++i)
	{
		uint32_t tri_index = m_tri_indices[i];
		Triangle& triangle = m_triangles[tri_index];

		node.bounds.pmin = Vec3Min(node.bounds.pmin, triangle.v0);
		node.bounds.pmax = Vec3Max(node.bounds.pmax, triangle.v0);

		node.bounds.pmin = Vec3Min(node.bounds.pmin, triangle.v1);
		node.bounds.pmax = Vec3Max(node.bounds.pmax, triangle.v1);

		node.bounds.pmin = Vec3Min(node.bounds.pmin, triangle.v2);
		node.bounds.pmax = Vec3Max(node.bounds.pmax, triangle.v2);
	}
}

void BVH::Subdivide(uint32_t node_index)
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
		if (m_triangles[m_tri_indices[i]].centroid.xyz[axis] < split_pos)
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

	Subdivide(left_child_index);
	Subdivide(right_child_index);
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
			IntersectTriangle(m_triangles[m_tri_indices[node.left_first + i]], ray);
		}
	}
	else
	{
		ray.payload.bvh_depth += 1;

		Intersect(ray, node.left_first);
		Intersect(ray, node.left_first + 1);
	}
}
