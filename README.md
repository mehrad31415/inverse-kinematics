# Kinematic Analysis of Human Fingers
This project is for the Robotics course. 

This project focuses on the kinematic analysis of human fingers, with the goal of accurately posing the fingers to reach specific target destinations or follow predetermined trajectories. The human hand consists of three main parts: the palm, wrist, and five distinct fingers. Each finger is composed of multiple joints and bones, primarily revolute joints that allow for bending and flexion motions.

The fingers have three main joints: the metacarpophalangeal (MCP), proximal interphalangeal (PIP), and distal interphalangeal (DIP) joints. The thumb differs from the other fingers as it has two hinge joints and two degrees of freedom. The other fingers have three hinge joints and three degrees of freedom. The rotations of the joints occur around the z-axis, and the angles are measured counterclockwise.

The project explores two scenarios: 

#### Unconstrained joint angles
In the unconstrained case, each joint angle can vary independently within specific ranges. The forward kinematics equations are derived to determine the position of the fingertip based on the joint angles. Additionally, the project investigates the configurations that place the fingertip on a pre-defined obstacle in space and explores the reachable positions of the fingertip without considering the obstacle.

#### Constrained joint angles. 
In this case, the joint angles are dependent upon each other where the precise mathematical dependencies are given in the report. The forward kinematics equations are derived. Likewise, the inverse kinematics problem is solved, that is, given the configuration T, we have determined the joint variables that result into this configuration. To solve the inverse kinematics problem, we have used the iterative approach. Lastly, we have considered the situation where the fingertip of the index finger is sliding on the surface of the object.

The above summary provides a brief overview of the project. For a comprehensive understanding and complete information, please refer to the detailed PDF documentation in the repository. The PDF documentation consists of mathematical equations, measurements, figures, and detailed thorough explanations.
