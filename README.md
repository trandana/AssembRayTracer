# AssembRayTracer

A real-time, interactive ray tracer written entirely in x64 assembly (MASM), using AVX-512 and all your CPU cores. No C runtime: the only imports are `kernel32`, `user32` and `gdi32`, and the executable is about 30 KB.

Fly around an abstract neon scene at 1280x720, rendering continuously until you close the window. Roughly 170-290 fps on a 16-core Ryzen 9 9950X3D.

## Controls

| Input | Action |
|---|---|
| Mouse | Look around |
| W / S | Move forward / back |
| A / D | Strafe left / right |
| E / Q | Move up / down |
| Shift | 3x speed |
| Esc / close button | Quit |

## Build and run

Requires Windows, Visual Studio 2022 (or Build Tools) with the C++ workload (it ships `ml64`), and a CPU with AVX-512 (Intel Skylake-X / Ice Lake and later, AMD Zen 4 and later).

- Command line: `build.bat run`
- Visual Studio: open `AssembRayTracer.sln`, pick **Release | x64**, Ctrl+F5.

## What it renders

- **Shapes:** spheres, axis-aligned boxes, diamonds (octahedra) and a floor. Boxes and diamonds share one slab-based intersector.
- **Glass:** refraction (Snell), Fresnel reflectance (Schlick), colour tint, total internal reflection, and light-passing shadows. Rays carry per-channel attenuation.
- **Style:** banded pink-to-indigo sky with a sun, toon-banded diffuse light, hard highlights, neon rim light, a glowing grid floor, mirror reflections, 2x2 anti-aliasing and gamma correction.

## How it is fast

- **Ray packets:** 16 rays travel together, one `zmm` register holds one component (x, y or z) of all 16 rays (structure-of-arrays). `k` mask registers handle rays that diverge.
- **Multithreading:** one thread per physical core takes 16x4-pixel work units from an atomic counter (`lock xadd`); the framebuffer is page-aligned so threads never share a cache line.
- **Table lookups:** object data lives in 16-entry tables read with `vpermps`, a 16-way in-register gather.
- **Small things:** `vrsqrt14ps` plus a Newton step, early-exit shadow rays, flush-to-zero, and iterative (not recursive) bounces.

## Layout

| Path | Purpose |
|---|---|
| `src/raytracer.asm` | The whole renderer |
| `build.bat` | Command-line build (finds Visual Studio via `vswhere`, runs `ml64` and `link`) |
| `AssembRayTracer.sln`, `.vcxproj` | Visual Studio project (MASM build customization) |

## Tweaking

- Scene: the `obj*` and `mat*` tables near the top of the data section (indices 0-3 spheres, 4-6 boxes, 7-8 diamonds, 9 floor). `matTr = 1` makes an object glass.
- Look: the palette block (`chkA*`, `lineR/G/B`, `kHor*`, `kSun*`, `kToon`, `kRim`, ...).
- Resolution: `IMG_W` (multiple of 16) and `IMG_H` (multiple of 4).
