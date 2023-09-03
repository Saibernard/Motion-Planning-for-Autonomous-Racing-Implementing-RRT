# Motion Planning for Autonomous Racing: Implementing RRT
This repository implements Motion Planning on F1 tenth car, focusing on Rapidly-exploring Random Trees (RRT) and their enhanced version, RRT*.


## Overview

### Autonomous Car Motion Planning

The primary aim of this project was to delve deep into the intricacies of motion planning, ensuring that an autonomous car can navigate a racing track with precision.

## Key Concepts Explored

### Motion Planning Basics:

- **Configuration Space vs. Workspace:** Distinguished between the advantages of planning within each space.
- **Free Space vs. Obstacle Space:** Recognized and differentiated between unoccupied spaces and areas with obstacles.
- **Occupancy Grids and Costmaps:** Learned and applied the concept of occupancy grids and costmaps to determine feasible paths.

### Motion Planning Algorithms:

- **Sampling-based Algorithms:** Primarily focused on Rapidly-exploring Random Trees (RRT).

## Implementation Highlights

### RRT Algorithm:

The core pseudocode was translated into executable code, using functions detailed in the given research paper. The unique aspect was adapting the traditional RRT to cater to the F1TENTH racing, where it acted as a local planner for obstacle avoidance.

### Occupancy Grid Creation:

A critical component was designing the occupancy grid to facilitate efficient collision checks. Both binary and probabilistic occupancy grids were options. The key was to optimize the grid's functionality, considering the lab's emphasis on speed and accuracy. A visualization tool ensured the grid's correct representation.

### Simulation & Real-world Testing:

The simulator was instrumental in validating the code's efficacy before transitioning to the car. The application on the car was done with incremental speed variations to ensure safety and effective algorithm evaluation.

![image](https://github.com/Saibernard/Motion-Planning-for-Autonomous-Racing-Implementing-RRT/assets/112599512/6532b0c1-fbe6-483e-acd1-ad7e0744cd4b)


### Trajectory Execution:

Upon RRT-path discovery, the trajectory was executed using the Pure Pursuit algorithm. The challenge lay in ensuring the car's smooth steering while maintaining a proactive approach to obstacle-avoidance.

## Visualization and Deployment

All the trees, trajectories, and algorithm performances were visualized in RVIZ. Successful GitHub code pushes and video demonstrations showcased the project's success.

## Simulation of the implementation of algorithm in autonomous car using Rviz

https://youtu.be/iQyuCCC_1Wo

## Conclusion

The project resulted in the development of a comprehensive motion planning algorithm capable of efficiently steering an autonomous car on a racing track.

