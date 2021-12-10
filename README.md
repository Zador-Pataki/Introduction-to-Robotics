# Introduction to Robotics 2020
Lab work and competition code for the course Introduction to Robotics (MEAM 520) at the University of Pennsylvania
## Competition (Winner)
The final project in this course was a group copetition (teams of four). Each team wrote a script to control a Lynx robot arm. The tasks the robots needed to be able to solve were grabbing both stationary and moving blocks, stacking these blocks without knocking over the stack, and avoiding collision with obstacles. In each round of the competition, the Lynx arms of both robots competed to get highest scoring cube stacks (scoring according to project descrption; see pdf under "Media and Documents".

### Media and Documents
- #### [Competition Blog Post](https://blog.seas.upenn.edu/virtual-robots-taking-risks-in-an-online-classroom/)
- #### [Project Description](https://github.com/Zador-Pataki/MEAM-520-2020/files/7690964/FinalProject.pdf)
- #### [Competition Report](https://github.com/Zador-Pataki/MEAM-520-2020/files/7690976/MEAM520_Final_Report.pdf)
- #### Final round of competition below (our arm is blue)

https://user-images.githubusercontent.com/55353663/145537919-da3384de-db31-4654-a87e-fe1205810c17.mp4

### Code (Competition Directory)
<details><summary>CONTROLLER.py</summary>is responsible for the general control of the robot throughout the entire process. It calls functions from other files appropriately. </details>
<details><summary>calculateFK.py</summary> is responsible for calculating the forward kinematics of our robot. This is how we infor the position of the robot in the euclidean space from the position of the robot in the state space.</details>
<details><summary>calculateIK.py</summary> is responsible for calculating the inverse kinematics of our robot. This is how we determine the required states of our robot in order to reach a given position and pose of the robot end-effector in the euclidean space.</details>
<details><summary>dynamic_tracking.py</summary> is responsible for handling the dynamic blocks (dynamic blocks are blocks on a turning table). The arm must be able ot follow a moving block without delay, given system delay, grab blocks while moving, determine which moving block to target given robot state -- not all positions on rotating table can be reached, i.e. blocks enter and exit feasible zones temporaly, and reset approach effectively: during gripping of moving blocks a lot can go on; robot must be able to identify when an error has been made, and must reevaluate its approach.</details>
<details><summary>final.py</summary> is a file needed for the competition</details>
<details><summary>stack.py</summary> is responsible for stacking a block given that the robot is already holding a block. It was critical that this is done precicely (otherwise we may knock over the existing stack) but also rapidly (a lot of time can be lost if we make the stacking precise by slowing down the process).</details>
<details><summary>static_block.py</summary> is responsible for grabbing the blocks on the static platforms. We observed that after grabbing the existing blocks on the rotating platform, we have extra time for pikcing up blocks from the static platforms, and so we could focus on grabbing the blocks on the static table in such a way that we can stack the blocks with the white face facing up (more points).</details>

## Lab work
Biweekly, we (groups of two) solved course tasks, which helped us develop the basic skills needed for the final competition. These tasks also covered the fundamental algorithms needed for the control of robot manipulators. 

### Lab 0 ([Lab0.pdf](https://github.com/Zador-Pataki/MEAM-520-2020/files/7691339/Lab0.pdf))
The purpose of Lab 0 was simply to set up the simulation environment.

### Lab 1 ([Lab1.pdf](https://github.com/Zador-Pataki/MEAM-520-2020/files/7691349/Lab1.pdf))
In Lab 1, we programmed the forward kinematics of our robot.

### Lab 2 ([Lab2.pdf](https://github.com/Zador-Pataki/MEAM-520-2020/files/7691353/Lab2.pdf))
In Lab 2, we programmed in the inverse kinematics of our robot.

### Lab 3 ([Lab3.pdf](https://github.com/Zador-Pataki/MEAM-520-2020/files/7691361/Lab3.pdf))
Lab 3 was about collision detection and avoidance. Solving htis task consisted of inferring the volume occupied by the robot arm in the euclidean space, given the state of the robot in the state space, and then identifying if this volume overlaps with objects in the map. Moreover, to avoid collision, we needed to infer if collision would occur if the robot followed some planned trajectory.

### Lab 4 ([Lab4.pdf](https://github.com/Zador-Pataki/MEAM-520-2020/files/7691398/Lab4.pdf))
In Lab 4, we calculated the forward and inverse kinematics of our robot. This consisted of inferring the voleocities of each joint, given join actuation, and inferring the joint actuation needed to achieve certain velocities in given joints.

### Lab 5 ([Lab5.pdf](https://github.com/Zador-Pataki/MEAM-520-2020/files/7691898/Lab5.pdf))
In Lab 5, our task was to control our robots using a potential field planner. The goal in such methods is to reach target states while avoiding obstacles, without deterministicely calculating a trajectory which will avoid obstacles, but instead updating the trajectory during the process. This is done by treating obstacles as protential sources (generate repolsive forces) and treating target states as potential sinks (generate attractive forces). The robot states tend directly towards the target until the repulsive forces from the obstacles are large enough (when the robot states are close enough to the obstacle) to divert the robot from the direct path. 


