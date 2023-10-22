# CPUGPUPathtracing

This is a project that I develop alongside the graphics advanced masterclass at BUAS, presented by Jacco Bikker. It will involve raytracing and pathtracing on the CPU, and eventually moving that to the GPU, as well as building your own acceleration structures.

Planned:
- Top-level and bottom-level acceleration structures with transforms
- BVH refitting
- Binned BVH building
- Additional improvements and optimizations to BVH building
- Additional optimizations for CPU raytracing and SIMD
- Microfacet BRDF (Physically-based rendering)
- GPU raytracing in compute (all software, no hardware)

Current: Path tracer (Work in progress)
Features:
- Soft shadows
- Bounding volume hierarchies (Triangle primitives only)
  - Mid point split
  - Surface area heuristic (SAH)
- Diffuse reflections
- Specular reflections
- Dielectrics
  - Beer's law (absorption)

Glass Dragon:

https://github.com/Contingencyy/GraphicsAdvancedMasterclass/assets/34250026/dd2dfc71-727f-4e18-b091-6d15b0cc88f1

Specular reflections in ground:

https://github.com/Contingencyy/GraphicsAdvancedMasterclass/assets/34250026/2b92ffef-3ef1-41b3-bf58-f2804c318472

BVH mid point split vs. SAH:

https://github.com/Contingencyy/GraphicsAdvancedMasterclass/assets/34250026/48768f8a-f8cf-48c0-b87f-1207447fb158



Previous: Whitted-style raytracer (Finished)
Features:
- Direct illumination from multiple light sources
  - Shadow rays
  - Distance attenuation
  - Lambert diffuse, N dot L
- Pure specular reflections (recursive)
- Dielectric materials like glass (recursive)
- Beer's law (absorption within medium)

https://github.com/Contingencyy/GraphicsAdvancedMasterclass/assets/34250026/45e9dc1d-ce37-4c9a-9801-f26bbefb4a2e
