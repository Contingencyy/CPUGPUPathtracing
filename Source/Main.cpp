#include "DX12.h"
#include "MathLib.h"
#include "Window.h"
#include "Input.h"
#include "ThreadPool.h"
#include "Primitives.h"
#include "Random.h"
#include "BVH.h"
#include "Util.h"
#include "GLTFLoader.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#ifdef CreateWindow
#undef CreateWindow
#endif

#ifdef LoadImage
#undef LoadImage
#endif

#ifdef OPAQUE
#undef OPAQUE
#endif

#ifdef TRANSPARENT
#undef TRANSPARENT
#endif

#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif

#ifdef near
#undef near
#endif

#ifdef far
#undef far
#endif

#include <stdint.h>
#include <stdexcept>
#include <chrono>
#include <array>
#include <vector>
#include <unordered_map>
#include <algorithm>

#include "imgui/imgui.h"
#include "imgui/imgui_impl_win32.h"
#include "imgui/imgui_impl_dx12.h"

/*
	
	Whitted-style raytracer (Finished):
	- Direct illumination of multiple light sources, takes into account:
		- Visibility
		- Distance attenuation
		- Shading model (N dot L for diffuse, or lambert)
	- Pure specular reflections with recursion
	- Dielectrics with fresnel with recursion
	- Beer's Law

	Cook raytracer (In-progress):
	- Area lights
	- Glossy reflections
	- Anti-aliasing
	- Motion blur
	- Depth of field

*/

static constexpr uint8_t MAX_RAY_DEPTH = 4;
static constexpr float RAY_REFLECT_NUDGE_MULTIPLIER = 0.001f;

struct Material
{
	Vec3 albedo = Vec3(0.0f);
	Vec3 emissive = Vec3(0.0f);
	float intensity = 1.0f;
	float specular = 0.0f;
	float refractivity = 0.0f;
	Vec3 absorption = Vec3(0.0f);
	float ior = 1.0f;
	bool is_light = false;
};

class Camera
{
public:
	Camera() = default;
	Camera(const Vec3& pos, const Vec3& view_dir, float fov, float aspect)
		: m_pos(pos), m_view_dir(view_dir), m_fov(Deg2Rad(fov)), m_aspect(aspect)
	{
		UpdateScreenPlane();
	}

	bool Update(float dt)
	{
		bool view_changed = false;

		// Camera look
		Vec2 mouse_move = Input::GetMouseMoveRel();

		// Camera movement
		float right_velocity = Input::GetInputAxis1D(Input::KeyCode_A, Input::KeyCode_D) * dt * m_speed;
		float up_velocity = Input::GetInputAxis1D(Input::KeyCode_Space, Input::KeyCode_LeftShift) * dt * m_speed;
		float forward_velocity = Input::GetInputAxis1D(Input::KeyCode_W, Input::KeyCode_S) * dt * m_speed;

		m_pos.x -= right_velocity;
		m_pos.y += up_velocity;
		m_pos.z -= forward_velocity;

		if (right_velocity != 0.0f || up_velocity != 0.0f || forward_velocity != 0.0f)
		{
			view_changed = true;
		}

		if (view_changed)
		{
			UpdateScreenPlane();
		}

		return view_changed;
	}

	Ray GetRay(float u, float v)
	{
		Vec3 pixel_pos = m_screen_plane.top_left +
			u * (m_screen_plane.top_right - m_screen_plane.top_left) +
			v * (m_screen_plane.bottom_left - m_screen_plane.top_left);

		return Ray(m_pos, Vec3Normalize(pixel_pos - m_pos));
	}

private:
	void UpdateScreenPlane()
	{
		m_screen_plane.center = m_pos + m_fov * m_view_dir;
		m_screen_plane.top_left = m_screen_plane.center + Vec3(-m_aspect, 1.0f, 0.0f);
		m_screen_plane.top_right = m_screen_plane.center + Vec3(m_aspect, 1.0f, 0.0f);
		m_screen_plane.bottom_left = m_screen_plane.center + Vec3(-m_aspect, -1.0f, 0.0f);
	}

private:
	Vec3 m_pos = Vec3(0.0f);
	Vec3 m_view_dir = Vec3(0.0f, 0.0f, -1.0f);

	float m_fov = Deg2Rad(60.0f);
	float m_aspect = 16.0f / 9.0f;

	float m_pitch = 0.0f;
	float m_yaw = 0.0f;
	float m_speed = 2.0f;

	struct ScreenPlane
	{
		Vec3 center = Vec3(0.0f);
		Vec3 top_left = Vec3(0.0f);
		Vec3 top_right = Vec3(0.0f);
		Vec3 bottom_left = Vec3(0.0f);
	} m_screen_plane;

};

enum DebugRenderView
{
	DebugRenderView_None,
	DebugRenderView_Diffuse,
	DebugRenderView_Specular,
	DebugRenderView_Refract,
	DebugRenderView_SurfaceNormal,
	DebugRenderView_SurfaceAlbedo,
	DebugRenderView_Depth,
	DebugRenderView_ViewDirection,
	DebugRenderView_BVHDepth,
	DebugRenderView_NumViews
};

const std::vector<const char*> debug_render_view_names =
{
	"None", "Diffuse", "Specular", "Refract", "Surface normal", "Surface albedo", "Depth", "View direction", "BVH Depth"
};

const std::vector<const char*> bvh_build_option_names =
{
	"Naive split", "SAH split intervals", "SAH split primitives"
};

struct Object
{
	Object(const char* name, const Mesh& mesh, uint32_t mat_index, BVH::BuildOption build_option)
		: name(name), material_index(mat_index)
	{
		bounding_volume_hierarchy.Build(mesh.vertices, mesh.indices, build_option);
	}

	BVH bounding_volume_hierarchy;
	uint32_t material_index = 0;
	const char* name = "";
};

struct Data
{
	// Render data
	std::vector<uint32_t> pixels;
	std::vector<Vec4> accumulator;
	uint32_t num_accumulated = 0;
	bool pause_rendering = false;

	std::vector<Object> objects;

	std::vector<Material> materials;
	Camera camera;
	DebugRenderView debug_view = DebugRenderView_None;
} static data;

static void ResetAccumulator()
{
	data.num_accumulated = 0;
	std::fill(data.accumulator.begin(), data.accumulator.end(), Vec4(0.0f));
}

void Update(float dt)
{
	Input::UpdateMousePosition();
	if (!ImGui::GetIO().WantCaptureMouse)
	{
		if (Input::IsKeyPressed(Input::KeyCode_LeftMouse))
		{
			Window::SetMouseCapture(true);
		}
	}
	if (Input::IsKeyPressed(Input::KeyCode_RightMouse))
	{
		Window::SetMouseCapture(false);
	}

	bool view_changed = data.camera.Update(dt);
	if (view_changed)
	{
		ResetAccumulator();
	}
}

void IntersectScene(Ray& ray)
{
	for (uint32_t obj_idx = 0; obj_idx < data.objects.size(); ++obj_idx)
	{
		bool hit = data.objects[obj_idx].bounding_volume_hierarchy.Traverse(ray);

		if (hit)
			ray.payload.obj_idx = obj_idx;
	}
}

Vec3 GetObjectSurfaceNormalAtPoint(PrimitiveType type, void* prim, const Vec3& point)
{
	switch (type)
	{
	case PrimitiveType_Triangle:
		return GetTriangleNormalAtPoint(*(Triangle*)prim, point);
	case PrimitiveType_Sphere:
		return GetSphereNormalAtPoint(*(Sphere*)prim, point);
	}
}

Vec4 TraceRay(Ray& ray, uint8_t depth)
{
	// Abort, max ray depth has been reached, return black
	if (depth == MAX_RAY_DEPTH)
		return Vec4(0.0f);

	IntersectScene(ray);

	// We have not hit anything, so we return black (or sky color)
	if (ray.payload.obj_idx == ~0u)
		return Vec4(0.0f);

	if (depth == 0 && data.debug_view == DebugRenderView_BVHDepth)
	{
		// Lerp from white to dark red based on bvh depth
		Vec4 bvh_depth_color = Vec4(1.0f);
		bvh_depth_color.xyz = Vec3Lerp(Vec3(0.0f, 1.0f, 0.0f), Vec3(1.0f, 0.0f, 0.0f), std::min(1.0f,
			(float)ray.payload.bvh_depth / (data.objects[ray.payload.obj_idx].bounding_volume_hierarchy.GetMaxDepth() + 1)));
		return bvh_depth_color;
	}

	// If we have hit a light source, we return its emissive color
	const Triangle& tri = data.objects[ray.payload.obj_idx].bounding_volume_hierarchy.GetTriangle(ray.payload.tri_idx);
	const Material& surface_material = data.materials[data.objects[ray.payload.obj_idx].material_index];

	if (surface_material.is_light)
	{
		return Vec4(surface_material.emissive * surface_material.intensity, 1.0f);
	}

	Vec4 final_color(0.0f, 0.0f, 0.0f, 1.0f);

	Vec3 surface_point = ray.origin + ray.t * ray.direction;
	Vec3 surface_normal = GetTriangleNormalAtPoint(tri, surface_point);

	Vec3 diffuse(0.0f);
	Vec3 specular(0.0f);
	Vec3 refract(0.0f);

	// ---------------------------------------------------------------------------------------------------------
	// Diffuse reflection

	Vec3 diffuse_dir = Util::UniformHemisphereSample(surface_normal);
	Ray diffuse_ray(surface_point + diffuse_dir * RAY_REFLECT_NUDGE_MULTIPLIER, diffuse_dir);
	float cosi = Vec3Dot(diffuse_dir, surface_normal);
	Vec4 irradiance = cosi * TraceRay(diffuse_ray, depth + 1);

	if (diffuse_ray.payload.obj_idx != ~0u)
	{
		Vec3 diffuse_brdf = surface_material.albedo * INV_PI;
		diffuse = 2.0f * PI * diffuse_brdf * irradiance.xyz;
	}

	// ---------------------------------------------------------------------------------------------------------
	// Final color and debug render views

	final_color.xyz = diffuse + specular + refract;

	if (depth == 0)
	{
		switch (data.debug_view)
		{
		case DebugRenderView_Diffuse:
		{
			final_color.xyz = diffuse;
		} break;
		case DebugRenderView_Specular:
		{
			final_color.xyz = specular;
		} break;
		case DebugRenderView_Refract:
		{
			final_color.xyz = refract;
		} break;
		case DebugRenderView_SurfaceNormal:
		{
			final_color.xyz = (surface_normal + 1.0f) * 0.5f;
		} break;
		case DebugRenderView_SurfaceAlbedo:
		{
			final_color.xyz = surface_material.albedo;
		} break;
		case DebugRenderView_Depth:
		{
			final_color.xyz = 0.1f * ray.t;
		} break;
		case DebugRenderView_ViewDirection:
		{
			final_color.xyz = (ray.direction + 1.0f) * 0.5f;
		} break;
		}
	}

	return final_color;
}

void Render()
{
	UVec2 framebuffer_size = Window::GetFramebufferSize();
	UVec2 job_size(16, 16);
	Vec2 inv_framebuffer_size(1.0f / framebuffer_size.x, 1.0f / framebuffer_size.y);

	data.num_accumulated++;

	auto trace_ray_thread_job = [&job_size, &framebuffer_size, &inv_framebuffer_size](ThreadPool::JobDispatchArgs args) {
		uint32_t first_x = (args.job_index * job_size.x) % framebuffer_size.x;
		uint32_t first_y = ((args.job_index * job_size.x) / framebuffer_size.x) * job_size.y;

		for (uint32_t y = first_y; y < first_y + job_size.y; ++y)
		{
			for (uint32_t x = first_x; x < first_x + job_size.x; ++x)
			{
				const float u = (float)x * inv_framebuffer_size.x;
				const float v = (float)y * inv_framebuffer_size.y;

				if (data.pause_rendering)
				{
					return;
				}
				
				Ray ray = data.camera.GetRay(u, v);
				Vec4 final_color = TraceRay(ray, 0);

				if (data.debug_view == DebugRenderView_None)
				{
					data.accumulator[y * framebuffer_size.x + x] += final_color;
					data.pixels[y * framebuffer_size.x + x] = Vec4ToUint(data.accumulator[y * framebuffer_size.x + x] / data.num_accumulated);
				}
				else
				{
					data.pixels[y * framebuffer_size.x + x] = Vec4ToUint(final_color);
				}
			}
		}
	};

	size_t pixel_count = framebuffer_size.x * framebuffer_size.y;
	uint32_t num_jobs = pixel_count / (job_size.y * job_size.x);

	ThreadPool::Dispatch(num_jobs, 16, trace_ray_thread_job);
	ThreadPool::WaitAll();
}

int main(int argc, char* argv[])
{
	Window::CreateWindowArgs window_args = {};
	window_args.width = 1280;
	window_args.height = 720;
	window_args.title = L"Graphics Advanced Masterclass";
	Window::Create(window_args);

	UVec2 framebuffer_size = Window::GetFramebufferSize();

	DX12::DX12InitArgs dx12_args = {};
	dx12_args.hWnd = Window::GetHandle();
	dx12_args.width = framebuffer_size.x;
	dx12_args.height = framebuffer_size.y;
	DX12::Init(dx12_args);

	ThreadPool::Init();

	data.pixels.resize(framebuffer_size.x * framebuffer_size.y);
	data.accumulator.resize(framebuffer_size.x * framebuffer_size.y);
	data.camera = Camera(Vec3(0.0f, 0.0f, 8.0f), Vec3(0.0f, 0.0f, -1.0f), 60.0f, (float)framebuffer_size.x / framebuffer_size.y);

	//data.materials.emplace_back(Vec3(0.4f, 0.1f, 0.1f), Vec3(0.0f), 0.0f, 0.0f, Vec3(0.0f), 1.0f, false);
	data.materials.emplace_back(Vec3(0.2f, 0.2f, 0.8f), Vec3(0.0f), 1.0f, 0.0f, 0.0f, Vec3(0.0f), 1.0f, false);
	data.materials.emplace_back(Vec3(0.4f), Vec3(0.0f), 1.0f, 0.0f, 0.0f, Vec3(0.0f), 1.0f, false);
	data.materials.emplace_back(Vec3(1.0f), Vec3(1.0f, 0.95f, 0.8f), 3.0f, 0.0f, 0.0f, Vec3(0.0f), 1.0f, true);
	//data.materials.emplace_back(Vec4(0.2f, 0.2f, 0.2f, 1.0f), 0.0f, 0.8f, Vec3(0.9f, 0.2f, 0.3f), 1.517f);

	// Load mesh and build its BVH
	Mesh dragon_mesh = GLTFLoader::Load("Assets/Models/Dragon/DragonAttenuation.gltf");
	data.objects.emplace_back("Dragon", dragon_mesh, 0, BVH::BuildOption_SAHSplitIntervals);
	
	Mesh light_mesh;
	light_mesh.indices.push_back(0);
	light_mesh.indices.push_back(1);
	light_mesh.indices.push_back(2);
	light_mesh.indices.push_back(2);
	light_mesh.indices.push_back(3);
	light_mesh.indices.push_back(0);
	light_mesh.vertices.push_back({ Vec3(-25.0f, 20.0f, 25.0f), Vec3(0.0f, -1.0f, 0.0f) });
	light_mesh.vertices.push_back({ Vec3(-25.0f, 20.0f, -25.0f), Vec3(0.0f, -1.0f, 0.0f) });
	light_mesh.vertices.push_back({ Vec3(25.0f, 20.0f, -25.0f), Vec3(0.0f, -1.0f, 0.0f) });
	light_mesh.vertices.push_back({ Vec3(25.0f, 20.0f, 25.0f), Vec3(0.0f, -1.0f, 0.0f) });
	data.objects.emplace_back("Light", light_mesh, 2, BVH::BuildOption_SAHSplitIntervals);

	Mesh ground_mesh;
	ground_mesh.indices.push_back(0);
	ground_mesh.indices.push_back(1);
	ground_mesh.indices.push_back(2);
	ground_mesh.indices.push_back(2);
	ground_mesh.indices.push_back(3);
	ground_mesh.indices.push_back(0);
	ground_mesh.vertices.push_back({ Vec3(-1000.0f, -3.0f, 1000.0f), Vec3(0.0f, 1.0f, 0.0f) });
	ground_mesh.vertices.push_back({ Vec3(-1000.0f, -3.0f, -1000.0f), Vec3(0.0f, 1.0f, 0.0f) });
	ground_mesh.vertices.push_back({ Vec3(1000.0f, -3.0f, -1000.0f), Vec3(0.0f, 1.0f, 0.0f) });
	ground_mesh.vertices.push_back({ Vec3(1000.0f, -3.0f, 1000.0f), Vec3(0.0f, 1.0f, 0.0f) });
	data.objects.emplace_back("Ground", ground_mesh, 1, BVH::BuildOption_SAHSplitIntervals);

	std::chrono::high_resolution_clock::time_point curr_time = std::chrono::high_resolution_clock::now(),
		last_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<float> delta_time = std::chrono::duration<float>(0.0f);

	while (!Window::ShouldClose())
	{
		curr_time = std::chrono::high_resolution_clock::now();
		delta_time = curr_time - last_time;

		ImGui_ImplWin32_NewFrame();
		ImGui_ImplDX12_NewFrame();
		ImGui::NewFrame();

		Window::PollEvents();
		Update(delta_time.count());
		Render();

		ImGui::Begin("General");
		ImGui::Text("Accumulated frames: %u", data.num_accumulated);
		if (ImGui::Checkbox("Pause rendering", &data.pause_rendering))
		{
			ResetAccumulator();
		}
		ImGui::Text("Frame time (CPU): %.3f ms", delta_time.count() * 1000.0f);
		if (ImGui::BeginCombo("Debug render view", debug_render_view_names[data.debug_view]))
		{
			for (size_t i = 0; i < DebugRenderView_NumViews; ++i)
			{
				bool is_selected = i == data.debug_view;
				if (ImGui::Selectable(debug_render_view_names[i], is_selected))
				{
					data.debug_view = (DebugRenderView)i;
				}

				if (is_selected)
				{
					ImGui::SetItemDefaultFocus();
				}
			}

			ImGui::EndCombo();
		}

		ImGui::SetNextItemOpen(true, ImGuiCond_Once);
		if (ImGui::CollapsingHeader("Scene"))
		{
			ImGui::Indent(10.0f);

			for (auto& obj : data.objects)
			{
				if (ImGui::CollapsingHeader(obj.name))
				{
					ImGui::Indent(10.0f);

					ImGui::Text("BVH Depth: %u", obj.bounding_volume_hierarchy.GetMaxDepth());
					if (ImGui::BeginCombo("Build options", bvh_build_option_names[obj.bounding_volume_hierarchy.GetBuildOption()]))
					{
						for (size_t i = 0; i < BVH::BuildOption_NumOptions; ++i)
						{
							bool is_selected = i == obj.bounding_volume_hierarchy.GetBuildOption();
							if (ImGui::Selectable(bvh_build_option_names[i], is_selected))
							{
								obj.bounding_volume_hierarchy.GetBuildOption() = (BVH::BuildOption)i;
							}

							if (is_selected)
							{
								ImGui::SetItemDefaultFocus();
							}
						}

						ImGui::EndCombo();
					}
					if (ImGui::Button("Rebuild BVH"))
					{
						obj.bounding_volume_hierarchy.Rebuild();
					}

					ImGui::Unindent(10.0f);
				}
			}

			ImGui::Unindent(10.0f);
		}
		ImGui::End();

		DX12::CopyToBackBuffer(data.pixels.data(), data.pixels.size() * sizeof(uint32_t));
		DX12::Present();

		ImGui::EndFrame();

		last_time = curr_time;
	}

	ThreadPool::Exit();
	DX12::Exit();
	Window::Destroy();

	return 0;
}