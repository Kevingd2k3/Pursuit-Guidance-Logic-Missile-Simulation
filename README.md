# 3D Kinematic Pursuit & Intercept Simulation

![Status](https://img.shields.io/badge/Status-Active-success)
![Python](https://img.shields.io/badge/Python-3.8%2B-blue)
![License](https://img.shields.io/badge/License-MIT-green)

A physics-based simulation of an aerial pursuit scenario involving an F-15 fighter jet and an interceptor missile. This project visualizes 3D kinematics, guidance logic, and collision events using Python's scientific stack.

##  Overview

This simulation demonstrates the implementation of **Pure Pursuit Guidance** logic in a 3D environment. It calculates trajectories for an evading target and a pursuing missile, handling 3D mesh manipulation (rotation/translation) in real-time to visualize the engagement.

**Key Features:**
* **3D Kinematics:** Simulates velocity, position, and orientation (Yaw/Pitch/Roll) in 3D space.
* **Pure Pursuit Logic:** Missile velocity vector updates dynamically to track the target line-of-sight.
* **STL Mesh Rendering:** Imports custom `.stl` 3D models for the aircraft and missile.
* **Dynamic Simplification:** Implements mesh decimation to maintain high FPS during animation.
* **Collision System:** "Splash" logic that detects intercepts, freezes trajectory history, and triggers visual destruction of targets.

##  Installation

1.  **Clone the repository**
    ```bash
    git clone [https://github.com/Kevingd2k3/Pursuit-Guidance-Logic-Missile-Simulation.git](https://github.com/Kevingd2k3/Pursuit-Guidance-Logic-Missile-Simulation.git)
    cd 3d-pursuit-simulation
    ```

2.  **Install Dependencies**
    This project relies on `numpy` for vector math, `matplotlib` for rendering, and `numpy-stl` for 3D mesh handling.
    ```bash
    pip install numpy matplotlib numpy-stl
    ```

3.  **Add 3D Models**
    [F15 STL File](F15.stl)
    [Missile STL File](Missile.stl)

## Controls

The simulation runs automatically upon launch.The camera angle is preset to an optimal viewing position.Use the standard Matplotlib toolbar to zoom, pan, or rotate the 3D view manually.

**Technical Details**

* **Guidance Law** 
The missile utilizes a Pure Pursuit algorithm where the velocity vector $\vec{V}_m$ is aligned with the Line of Sight (LOS) vector to the target $\vec{P}_t$:
$$ \vec{V}{new} = V{mag} \cdot \frac{\vec{P}_t - \vec{P}_m}{||\vec{P}_t - \vec{P}_m||} $$

* **Coordinate Transformation**
To visualize the 3D models correctly, the static mesh is rotated using a combined Euler rotation matrix derived from the velocity vector:
$$ R = R_z(\psi) \cdot R_y(\theta) $$Where yaw ($\psi$) and pitch ($\theta$) are calculated from the current velocity components $(v_x, v_y, v_z)$.

##  Acknowledgements

* **Original Inspiration:** Based on the kinematic concepts from [Mention Original Repo Name/Link Here if applicable].
* **Libraries:** Built using Matplotlib and NumPy-STL.

##  License

This project is licensed under the MIT License - see the LICENSE file for details.

