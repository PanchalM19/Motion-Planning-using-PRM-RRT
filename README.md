# Motion-Planning-using-PRM-RRT

## Overview
In this project, we delved into the domain of sampling-based motion planning, implementing two prominent algorithms, Probabilistic Roadmap (PRM) and Rapidly Exploring Random Tree (RRT), for a 4-DOF 2-link robotic arm navigating around a spherical obstacle. The project was structured around a series of assignments focusing on collision checking, roadmap generation, path finding, and optimization of generated paths.

### M0: Understanding the Environment
* Explored the provided code and visualized the robotic arm in various configurations.
* Examined collision checking functions and identified potential issues, setting the groundwork for subsequent improvements.

### M1: Sampling Configuration Space
* Generated uniformly distributed samples from configuration space, considering joint limits while allowing collisions.

### M2: Probabilistic Roadmap (PRM) Construction
* Implemented the PRM algorithm to construct a roadmap, ensuring the graph comprised collision-free configurations within joint limits.
* Utilized an adjacency matrix to represent distances between vertices, considering neighbors for effective roadmap navigation.

### M3: Path Finding Using PRM
* Leveraged the constructed roadmap to find a collision-free path from the start to the goal configuration.
* Applied appropriate points for seamless entry and exit from the roadmap.

### M4: Rapidly Exploring Random Tree (RRT) Implementation
* Developed the RRT algorithm to discover a collision-free path by selecting suitable hyperparameter values for step size and goal sampling frequency.
* Traversed the constructed tree to recover the actual optimized path.

### M5: Path Optimization with RRT
* Improved the RRT-generated path by removing unnecessary waypoints, ensuring smoother and more efficient motion.

M6: Challenge Your Algorithm
* Concluded the project by watching the algorithms in action on more challenging motion planning problems, exploring possibilities to make them even more robust.
  
This project showcases proficiency in MATLAB motion planning techniques, offering valuable insights into the complexities and optimizations involved in sampling-based motion planning for robotic systems.
