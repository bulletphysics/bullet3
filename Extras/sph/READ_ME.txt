
FLUIDS v.1 - SPH Fluid Simulator for CPU and GPU
Copyright (C) 2008. Rama Hoetzlein, http://www.rchoetzlein.com

A fast CPU and GPU fluid simulator. See website above for details.

Notes 
(April 2009)
-------------------

- Press 'h' for help screen. This shows keyboard cmds available.

- By default, GPU simulation is off.
  When running the fluid_gpu.exe, press 'g' to start/stop GPU simulation.

- Disabling shadows in common_defs.h will greatly speed up the simulation
  (You can also press 's' to render without shadows)

- As of April 2009, CUDA builds only under Visual Studio 2005, not VS 2008.
  To enable the CUDA build in VS2005, set the BUILD_CUDA define in common_defs.h
  Be sure this is turned off if you build with VS2008.

- The GPU integrator is not yet complete. Integration always takes place on the CPU, in both CPU and GPU modes. (As a result, this forces a bus transfer to and from the GPU per cycle. Once the integrator is finished, GPU simulation performance should increase significantly.)

- Occassionally, the GPU simulation with crash cuda, causing the screen to blink and particles to move randomly. This is believed to be due to a not-yet-found out of bounds condition.

ZLib License
-------------------
This software is provided 'as-is', without any express or implied  warranty.  In no event will the authors be held liable for any damages arising from the use of this software.

Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.