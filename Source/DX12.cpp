#include "DX12.h"

#include <d3d12.h>
#include <dxgi1_6.h>
#include <dxgidebug.h>
#include <d3d12shader.h>
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

#include <stdexcept>
#define DX_EXCEPT(msg) throw std::runtime_error(msg)

inline void GetHRESULTMessage(HRESULT hr)
{
	char* msg = nullptr;
	FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL, hr, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (char*)&msg, 0, NULL);
	printf(msg);
	DX_EXCEPT(msg);
}

#define DX_CHECK(hr) if (FAILED(hr)) { GetHRESULTMessage(hr); }

template<typename T>
inline intptr_t AlignUp(T x, uint32_t align)
{
	return ((intptr_t)(x) + ((align) - 1) & (-(intptr_t)(align)));
}

namespace DX12
{

	static constexpr uint32_t DX_BACK_BUFFER_COUNT = 2;

	struct Data
	{
		IDXGIAdapter4* dxgi_adapter = nullptr;
		ID3D12Device8* d3d_device = nullptr;

		ID3D12CommandQueue* d3d_command_queue = nullptr;
		IDXGISwapChain4* dxgi_swap_chain = nullptr;
		ID3D12Resource* back_buffers[DX_BACK_BUFFER_COUNT];
		uint32_t current_back_buffer_index = 0;
		ID3D12Fence* fence = nullptr;
		HANDLE fence_event = nullptr;
		uint64_t fence_value = 0;
		bool tearing_supported = false;
		bool vsync_enabled = false;

		ID3D12CommandAllocator* d3d_command_allocator = nullptr;
		ID3D12GraphicsCommandList6* d3d_command_list = nullptr;

		ID3D12Resource* upload_buffer = nullptr;
		void* upload_buffer_ptr = nullptr;
	} static data;

	D3D12_RESOURCE_BARRIER TransitionBarrier(ID3D12Resource* resource, D3D12_RESOURCE_STATES state_before, D3D12_RESOURCE_STATES state_after)
	{
		D3D12_RESOURCE_BARRIER barrier = {};
		barrier.Type = D3D12_RESOURCE_BARRIER_TYPE_TRANSITION;
		barrier.Transition.pResource = resource;
		barrier.Transition.Subresource = 0;
		barrier.Transition.StateBefore = state_before;
		barrier.Transition.StateAfter = state_after;
		barrier.Flags = D3D12_RESOURCE_BARRIER_FLAG_NONE;

		return barrier;
	}

	void Init(const DX12InitArgs& args)
	{
		// Enable debug layer
		ID3D12Debug* debug_controller = nullptr;
		DX_CHECK(D3D12GetDebugInterface(IID_PPV_ARGS(&debug_controller)));
		debug_controller->EnableDebugLayer();

		// Enable GPU validation
		/*ID3D12Debug1* debug_controller1;
		DX_CHECK(debug_controller->QueryInterface(IID_PPV_ARGS(&debug_controller1)))
		{
			debug_controller1->SetEnableGPUBasedValidation(true);
		}*/

		uint32_t factory_create_flags = 0;
#ifdef _DEBUG
		factory_create_flags |= DXGI_CREATE_FACTORY_DEBUG;
#endif

		// Create DXGI factory
		IDXGIFactory7* dxgi_factory = nullptr;
		DX_CHECK(CreateDXGIFactory2(DXGI_CREATE_FACTORY_DEBUG, IID_PPV_ARGS(&dxgi_factory)));

		// Select adapter
		D3D_FEATURE_LEVEL d3d_min_feature_level = D3D_FEATURE_LEVEL_12_0;
		IDXGIAdapter1* dxgi_adapter = nullptr;
		size_t max_dedicated_video_memory = 0;

		for (size_t adapter_idx = 0; dxgi_factory->EnumAdapters1(adapter_idx, &dxgi_adapter) != DXGI_ERROR_NOT_FOUND; ++adapter_idx)
		{
			DXGI_ADAPTER_DESC1 adapter_desc = {};
			DX_CHECK(dxgi_adapter->GetDesc1(&adapter_desc));

			if ((adapter_desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE) == 0 &&
				SUCCEEDED(D3D12CreateDevice(dxgi_adapter, d3d_min_feature_level, __uuidof(ID3D12Device), nullptr)) &&
				adapter_desc.DedicatedVideoMemory > max_dedicated_video_memory)
			{
				max_dedicated_video_memory = adapter_desc.DedicatedVideoMemory;
				DX_CHECK(dxgi_adapter->QueryInterface(IID_PPV_ARGS(&data.dxgi_adapter)));
			}
		}

		// Create device
		DX_CHECK(D3D12CreateDevice(data.dxgi_adapter, d3d_min_feature_level, IID_PPV_ARGS(&data.d3d_device)));

		// Set info queue behavior
		ID3D12InfoQueue* info_queue = nullptr;
		DX_CHECK(data.d3d_device->QueryInterface(IID_PPV_ARGS(&info_queue)));
		DX_CHECK(info_queue->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_CORRUPTION, TRUE));
		DX_CHECK(info_queue->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_ERROR, TRUE));
		DX_CHECK(info_queue->SetBreakOnSeverity(D3D12_MESSAGE_SEVERITY_WARNING, TRUE));

		// Check for tearing support
		BOOL allow_tearing = FALSE;
		DX_CHECK(dxgi_factory->CheckFeatureSupport(DXGI_FEATURE_PRESENT_ALLOW_TEARING, &allow_tearing, sizeof(BOOL)));
		data.tearing_supported = (allow_tearing == TRUE);

		// Create swap chain command queue
		D3D12_COMMAND_QUEUE_DESC queue_desc = {};
		queue_desc.Type = D3D12_COMMAND_LIST_TYPE_DIRECT;
		queue_desc.Priority = D3D12_COMMAND_QUEUE_PRIORITY_NORMAL;
		queue_desc.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE;
		queue_desc.NodeMask = 0;
		DX_CHECK(data.d3d_device->CreateCommandQueue(&queue_desc, IID_PPV_ARGS(&data.d3d_command_queue)));

		// Create swap chain
		DXGI_SWAP_CHAIN_DESC1 swap_chain_desc = {};
		swap_chain_desc.Width = args.width;
		swap_chain_desc.Height = args.height;
		swap_chain_desc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
		swap_chain_desc.Stereo = FALSE;
		swap_chain_desc.SampleDesc = { 1, 0 };
		swap_chain_desc.BufferUsage = DXGI_USAGE_BACK_BUFFER;
		swap_chain_desc.BufferCount = DX_BACK_BUFFER_COUNT;
		swap_chain_desc.Scaling = DXGI_SCALING_STRETCH;
		swap_chain_desc.SwapEffect = DXGI_SWAP_EFFECT_FLIP_DISCARD;
		swap_chain_desc.AlphaMode = DXGI_ALPHA_MODE_UNSPECIFIED;
		swap_chain_desc.Flags = data.tearing_supported ? DXGI_SWAP_CHAIN_FLAG_ALLOW_TEARING : 0;

		IDXGISwapChain1* dxgi_swap_chain = nullptr;
		DX_CHECK(dxgi_factory->CreateSwapChainForHwnd(data.d3d_command_queue, args.hWnd, &swap_chain_desc, nullptr, nullptr, &dxgi_swap_chain));
		DX_CHECK(dxgi_swap_chain->QueryInterface(&data.dxgi_swap_chain));
		DX_CHECK(dxgi_factory->MakeWindowAssociation(args.hWnd, DXGI_MWA_NO_ALT_ENTER));
		data.current_back_buffer_index = data.dxgi_swap_chain->GetCurrentBackBufferIndex();

		for (size_t i = 0; i < DX_BACK_BUFFER_COUNT; ++i)
		{
			data.dxgi_swap_chain->GetBuffer(i, IID_PPV_ARGS(&data.back_buffers[i]));
		}

		// Create command allocator and command list
		DX_CHECK(data.d3d_device->CreateCommandAllocator(D3D12_COMMAND_LIST_TYPE_DIRECT, IID_PPV_ARGS(&data.d3d_command_allocator)));
		DX_CHECK(data.d3d_device->CreateCommandList(0, D3D12_COMMAND_LIST_TYPE_DIRECT, data.d3d_command_allocator, nullptr, IID_PPV_ARGS(&data.d3d_command_list)));

		// Create fence and fence event
		DX_CHECK(data.d3d_device->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(&data.fence)));
		HANDLE fence_event = ::CreateEvent(NULL, FALSE, FALSE, NULL);
		if (!fence_event)
		{
			DX_EXCEPT("Failed to create fence event handle");
		}

		// Create upload buffer
		D3D12_HEAP_PROPERTIES heap_props = {};
		heap_props.Type = D3D12_HEAP_TYPE_UPLOAD;

		D3D12_RESOURCE_DESC resource_desc = {};
		resource_desc.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
		resource_desc.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
		resource_desc.Alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT;
		resource_desc.Format = DXGI_FORMAT_UNKNOWN;
		resource_desc.Width = args.width * args.height * 4;
		resource_desc.Height = 1;
		resource_desc.DepthOrArraySize = 1;
		resource_desc.MipLevels = 1;
		resource_desc.SampleDesc.Count = 1;
		resource_desc.Flags = D3D12_RESOURCE_FLAG_NONE;

		DX_CHECK(data.d3d_device->CreateCommittedResource(&heap_props, D3D12_HEAP_FLAG_NONE,
			&resource_desc, D3D12_RESOURCE_STATE_GENERIC_READ, nullptr, IID_PPV_ARGS(&data.upload_buffer)));
		data.upload_buffer->Map(0, nullptr, &data.upload_buffer_ptr);
	}

	void Exit()
	{
		data.upload_buffer->Unmap(0, nullptr);
		::CloseHandle(data.fence_event);
	}

	void CopyToBackBuffer(uint32_t* pixels, size_t num_bytes)
	{
		ID3D12Resource* back_buffer = data.back_buffers[data.current_back_buffer_index];

		memcpy(data.upload_buffer_ptr, pixels, num_bytes);

		D3D12_RESOURCE_BARRIER copy_dest_barrier = TransitionBarrier(back_buffer, D3D12_RESOURCE_STATE_PRESENT, D3D12_RESOURCE_STATE_COPY_DEST);
		data.d3d_command_list->ResourceBarrier(1, &copy_dest_barrier);

		uint32_t bpp = 4;

		D3D12_RESOURCE_DESC dst_desc = back_buffer->GetDesc();
		D3D12_SUBRESOURCE_FOOTPRINT dst_footprint = {};
		dst_footprint.Format = back_buffer->GetDesc().Format;
		dst_footprint.Width = back_buffer->GetDesc().Width;
		dst_footprint.Height = back_buffer->GetDesc().Height;
		dst_footprint.Depth = 1;
		dst_footprint.RowPitch = AlignUp(back_buffer->GetDesc().Width * bpp, D3D12_TEXTURE_DATA_PITCH_ALIGNMENT);

		D3D12_PLACED_SUBRESOURCE_FOOTPRINT src_footprint = {};
		src_footprint.Footprint = dst_footprint;
		src_footprint.Offset = 0;

		uint8_t* src_ptr = (uint8_t*)pixels;
		uint8_t* dst_ptr = (uint8_t*)data.upload_buffer_ptr;
		uint32_t dst_pitch = dst_footprint.RowPitch;

		for (uint32_t y = 0; y < back_buffer->GetDesc().Height; ++y)
		{
			memcpy(dst_ptr, src_ptr, dst_pitch);
			src_ptr += back_buffer->GetDesc().Width * bpp;
			dst_ptr += dst_pitch;
		}

		D3D12_TEXTURE_COPY_LOCATION src_loc = {};
		src_loc.pResource = data.upload_buffer;
		src_loc.PlacedFootprint = src_footprint;
		src_loc.Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT;

		D3D12_TEXTURE_COPY_LOCATION dst_loc = {};
		dst_loc.pResource = back_buffer;
		dst_loc.SubresourceIndex = 0;
		dst_loc.Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX;

		data.d3d_command_list->CopyTextureRegion(&dst_loc, 0, 0, 0, &src_loc, nullptr);

		D3D12_RESOURCE_BARRIER present_barrier = TransitionBarrier(back_buffer, D3D12_RESOURCE_STATE_COPY_DEST, D3D12_RESOURCE_STATE_PRESENT);
		data.d3d_command_list->ResourceBarrier(1, &present_barrier);

		data.d3d_command_list->Close();
		ID3D12CommandList* const command_lists[] = { data.d3d_command_list };
		data.d3d_command_queue->ExecuteCommandLists(1, command_lists);
	}

	void Present()
	{
		// Present
		uint32_t sync_interval = data.vsync_enabled ? 1 : 0;
		uint32_t present_flags = data.tearing_supported && !data.vsync_enabled ? DXGI_PRESENT_ALLOW_TEARING : 0;
		DX_CHECK(data.dxgi_swap_chain->Present(sync_interval, present_flags));

		// Wait for frame to complete rendering
		data.fence_value++;
		data.d3d_command_queue->Signal(data.fence, data.fence_value);
		if (data.fence->GetCompletedValue() < data.fence_value)
		{
			DX_CHECK(data.fence->SetEventOnCompletion(data.fence_value, data.fence_event));
			::WaitForSingleObjectEx(data.fence_event, UINT32_MAX, FALSE);
		}
		data.current_back_buffer_index = data.dxgi_swap_chain->GetCurrentBackBufferIndex();

		// Reset the command allocator and command list
		data.d3d_command_allocator->Reset();
		data.d3d_command_list->Reset(data.d3d_command_allocator, nullptr);
	}

}
