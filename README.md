# CompGeom
Computational geometry sandbox

## 1. Features

Deformable/dynamic mesh models:
* Mass-spring systems
* As-Rigid-As-Possible (ARAP) surface modeling [1]
* Finite Element Method (2D triangular elements)
* Position Based Dynamics (PBD)
* ...

Parametric surfaces:
* Bezier surfaces
* B-splines
* Thin Plate Spline (TPS) [2]
* ...

## 2. Sources

* [1] https://igl.ethz.ch/projects/ARAP/index.php
* [2] https://elonen.iki.fi/code/tpsdemo/


## 3. External dependencies

Requires the installation of Vulkan SDK: https://vulkan.lunarg.com/sdk

All external dependencies are open-source libraries.

GLtools.h and other libraries are provided in the [libs](https://github.com/ludoBcg/libs) repository. 

External dependencies used for this project are:

* [GLM (OpenGL Mathematics)](https://github.com/g-truc/glm)

* [GLFW (Graphics Library Framework)](https://www.glfw.org/)

* [Eigen](https://gitlab.com/libeigen/eigen)
