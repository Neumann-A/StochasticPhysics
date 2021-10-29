**CI status**  
[![Windows(clang-cl)](https://github.com/Neumann-A/StochasticPhysics/actions/workflows/ci.windows.yml/badge.svg)](https://github.com/Neumann-A/StochasticPhysics/actions/workflows/ci.windows.yml)
[![Linux(g++/clang++)](https://github.com/Neumann-A/StochasticPhysics/actions/workflows/ci.linux.yml/badge.svg)](https://github.com/Neumann-A/StochasticPhysics/actions/workflows/ci.linux.yml)  
dev branch  
[![Windows(clang-cl)](https://github.com/Neumann-A/StochasticPhysics/actions/workflows/ci.windows.yml/badge.svg?branch=dev)](https://github.com/Neumann-A/StochasticPhysics/actions/workflows/ci.windows.yml)
[![Linux(g++/clang++)](https://github.com/Neumann-A/StochasticPhysics/actions/workflows/ci.linux.yml/badge.svg?branch=dev)](https://github.com/Neumann-A/StochasticPhysics/actions/workflows/ci.linux.yml)

# Introduction

This project is aimed to perform stochastic simulations of physical processes. Currently the project is focused to perform those simulations for single domain magnetic particles. 

<hr>

### Getting Started

<details>
<summary>Installation prerequisite</summary>

 1. [CMake](https://cmake.org/) 
 2. [MATLAB](www.mathworks.com) (Linux: Needs to be on PATH for CMake to find it!)
 3. C++ compiler e.g. clang(-cl) from [LLVM](https://llvm.org/) (don't use MSVC if you care about performance). 
 
</details>

<details>
<summary>Installation process</summary>

 1. Get the code (executed from `<basedir>`)
    * `git clone https://github.com/Neumann-A/StochasticPhysics.git <srcdir>`
    * `cd <srcdir>`
    * setup git submodules via `git submodule init` and `git submodule update`
 3. setup vcpkg.
    1. `git clone https://github.com/microsoft/vcpkg.git` (either in `<srcdir>` or `<basedir>`)
    2. Bootstrap vcpkg. `cd vcpkg`
       * Windows: `bootstrap.bat -disableMetrics -win64` 
       * Linux: `./bootstrap.sh -disableMetrics` (optional: `-useSystemBinaries`)
    3. Install dependencies via vcpkg
        * `vcpkg install serar pcg-cpp eigen3 boost-random --overlay-ports=<srcdir>/vcpkg-ports`
            * On Windows: use `--triplet x64-windows` or `--triplet x64-windows-static`
 4. configure with cmake e.g. `cmake -G "Ninja" -DCMAKE_BUILD_TYPE=Release -S <path-to-source> -B <path-to-build>`. Available build options are: `VCPKG_TARGET_TRIPLET`, `BUILD_TESTING`, `BUILD_BENCHMARKS`
    * You might need to set `-DVCPKG_TARGET_TRIPLET=<triplet>`
    * Other options (the default should be fine) `Simulation_PCG`, `Simulation_Boost_Random`,`Simulation_WITH_GSL_Solvers` (requires GSL), `Simulation_WITH_ImplicitMidpoint`
 5. build/install with cmake e.g. `cmake --build [<options>] -B <path-to-build>`
    * `--target SimulationApplicationNew` or `StochasticPhysics`
    * `--config Release`
 
</details>

<details>
<summary>Running test</summary>

 * Same as installation process
 * Tests are located in the Tests subfolder
 * Not all tests are designed to be succesful. 
 * List of availale test targets:
   * Random_State_Init_Test
   * Implicit_solver_Test
   * Neel_Problem_Test
   * Neel_Spherical_Test
   * BrownAndNeel_Relaxation_Test (TODO: remove Relaxation)
   * NeelSpherical_BrownEuler_Relaxation_Test (TODO: remove Relaxation)
 
</details>

 ### Running Simulations

Execute either `StochasticPhysics` or `SimulationApplicationNew` it will run a dummy simulation generating a pair of `*.ini` configfiles. Those configfiles can be modified and passed to the applications via:

 * `StochasticPhysics -parfile:<SimulationSettings>`
 * `StoPhysApp_MultiArch --parameter_file=<SimulationSettings>`

The main difference between the two applications is that the former is a single architecture executable while the later is a multi architecture executable (AVX, AVX2, AVX512). The single architecture executable will probably be removed in the future. 

Example for other config files can be found in the examples folder. 

Running the executables on Linux might require extra setup of `LD_LIBRARY_PATH`. The Code is optimized and tested mainly on Windows systems. 
 
 ### Citing
 Since there is currently no publication covering the internals of the code, citing is done by referencing the github url and the used git commit id. This is done 

 ### Publications/Poster/Talks using this Code

 <details>
 <summary>Universität zu Lübeck</summary>
 A. Neumann & T. M. Buzug
 Stochastic Simulations of Magnetic Particles: Comparison of Different Methods
 8th International Workshop on Magnetic Particle Imaging 213, Hamburg (Deutschland) (2018).

 A. Neumann, S. Draack, F. Ludwig & T. M. Buzug
 Parameter estimations of magnetic particles: A comparison between measurements and simulations
 9th International Workshop on Magnetic Particle Imaging 79, New York (USA) (2019)

 T. Klemme, T. M. Buzug & A. Neumann
 Exploring parameters of magnetic particles in 1D field excitation
 9th International Workshop on Magnetic Particle Imaging 189, New York (USA) (2019)
 T. Klemme, T. M. Buzug & A. Neumann
 Exploring Parameters of Magnetic Particles in 1D Field Excitation
 International Journal on Magnetic Particle Imaging, 6(2), 2004001, (2020)

 A. Neumann, & T. M. Buzug
 Simulations of magnetic particles with arbitrary anisotropies
 International Journal on Magnetic Particle Imaging, 6(2) Suppl. 1, 2009032, (2020)

 </details>

 <details>
 <summary>Others</summary>
 None yet
 </details>

 #### Regarding API/ABI stability
 No gurantees on API or ABI stability are given. 