# Fourbar GUI (C++ / Dear ImGui / WebAssembly)

A C++ port of the MATLAB elliptical-trainer linkage tools in the repository root,
with a [Dear ImGui](https://github.com/ocornut/imgui) + [ImPlot](https://github.com/epezent/implot)
GUI that runs natively (GLFW + OpenGL 3) and in the browser (Emscripten / WebAssembly).

Online version: https://robincpc.github.io/Fourbar_GUI/ (published by the `C++ app` workflow).

## Features

* **Four-bar tab** (ports `fourbarGUI.m` and `FourAnalysis.m`)
  * Edit frame, crank, coupler, rocker and coupler-point dimensions, frame and beta angles,
    crank torque, and the assembly mode (+1 / -1); everything updates live.
  * Animated linkage with the paths of the pedal point P and joint B.
  * Moment arm and minimum pedal force vs. crank angle, and the summary values of the MATLAB `E` vector.
* **Eight-bar tab** (ports `EightbarAnalysis.m`)
  * Edit all link lengths and angles; animated linkage with the path of P.
  * theta5 vs. crank angle and the summary values of the MATLAB `E` vector.
* Play/pause, animation speed and a crank-angle slider replace the MATLAB Animate button and
  the `y/n` / `+/-/=` prompts.
* Plots: drag to pan, scroll to zoom, double-click to fit.

## Layout

```
cpp/
  src/linkage.{h,cpp}     kinematics (no GUI dependencies)
  src/app.{h,cpp}         ImGui/ImPlot user interface
  src/main.cpp            GLFW + OpenGL 3 entry point (also used by the wasm build)
  tests/test_linkage.cpp  unit tests against values from the original MATLAB scripts
  third_party/            imgui (docking branch) and implot as git submodules
  Makefile                native build + tests
  Makefile.emscripten     WebAssembly build
  web_shell.html          HTML page template for the wasm build
```

## Build

Fetch the submodules first:

```sh
git submodule update --init --recursive
cd cpp
```

**Unit tests** (only a C++17 compiler needed):

```sh
make test
```

**Native app** (needs GLFW: `apt-get install libglfw3-dev` / `brew install glfw`):

```sh
make
./fourbar_gui
```

**WebAssembly** (needs the [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html)):

```sh
source /path/to/emsdk/emsdk_env.sh
make -f Makefile.emscripten          # produces web/index.html, web/index.js, web/index.wasm
make -f Makefile.emscripten serve    # serves web/ on http://localhost:8000
```

## Notes on the port

* The math follows the MATLAB code, including the four-bar crank sweep (180 deg down to -179 deg),
  the eight-bar sweep (0 to 359 deg), and the fixed eight-bar values r12 = 55, r13 = r1 + 6,
  r14 = r4 + 59. `r9` from `EightbarAnalysis.m` is not used there, so it is not an input here either.
* The tests compare the summary values and sample poses with results from running the original
  scripts (default inputs) in GNU Octave.
* The four-bar moment arm and force use the coupler angle of the selected assembly mode.
  `FourAnalysis.m` only supported mode +1, where the results are identical.
