This is a custom 2D Rigid-body Physics Engine
---

Implements convex polygon collision detection via SAT, impulse-based resolution (including rotation and Coulomb friction), and a broadphase pipeline using AABBs + spatial partitioning for performance.

## Features
- [x] SAT convex polygon collision detection  
- [x] Impulse resolution with angular dynamics  
- [x] Coulomb friction (static/dynamic)  
- [x] Broadphase AABB culling  
- [x] Spatial partitioning 


## Installation
Clone the repository
```bash
git clone --recurse-submodules https://github.com/AndrewGossenPerez/2DPhysicsEngine.git
mkdir build
cd build
cmake ..
make
```
If you've already cloned without submodules
```bash 
git submodule update --init --recursive
```

**Demo Videos**
---

https://github.com/user-attachments/assets/7ada4b5e-ff8c-4732-9831-de9ed36a6977

https://github.com/user-attachments/assets/12808075-fe2f-4803-b852-b403fb83c422




