# MEAM-520-2020
Lab work and competition code for the course Introduction to Robotics (MEAM 520) at the University of Pennsylvania
## Competition (Winner)
The final project in this course was a group copetition (teams of 4). Each team wrote a script to control a Lynx robot arm. The tasks the robots needed to be able to solve were grabbing both stationary and moving blocks, stacking these blocks without knocking over the stack, and avoiding collision with obstacles. In each round of the competition, the Lynx arms of both robots competed to get highest scoring cube stacks (scoring according to project descrption; see pdf under "Media and Documents".

### Media and Documents
- [Competition Blog Post](https://blog.seas.upenn.edu/virtual-robots-taking-risks-in-an-online-classroom/)
- [Project Description](https://github.com/Zador-Pataki/MEAM-520-2020/files/7690964/FinalProject.pdf)
- [Competition Report](https://github.com/Zador-Pataki/MEAM-520-2020/files/7690976/MEAM520_Final_Report.pdf)
- Final round of competition below (our arm is blue)

https://user-images.githubusercontent.com/55353663/145537919-da3384de-db31-4654-a87e-fe1205810c17.mp4

### Code (Competition Directory)
<details><summary>CONTROLLER.py</summary>is responsible for the general control of the robot throughout the entire process. It calls functions from other files appropriately. </details>
<details><summary>calculateFK.py</summary> is responsible for calculating the forward kinematics of our robot. This is how we infor the position of the robot in the euclidean space from the position of the robot in the state space.</details>
<details><summary>calculateIK.py</summary> is responsible for calculating the inverse kinematics of our robot. This is how we determine the required states of our robot in order to reach a given position and pose of the robot end-effector in the euclidean space.</details>
<details><summary>dynamic_tracking.py</summary> is responsible for handling the dynamic blocks (dynamic blocks are blocks on a turning table). The arm must be able ot follow a moving block without delay, given system delay, grab blocks while moving, determine which moving block to target given robot state -- not all positions on rotating table can be reached, i.e. blocks enter and exit feasible zones temporaly, and reset approach effectively: during gripping of moving blocks a lot can go on; robot must be able to identify when an error has been made, and must reevaluate its approach.</details>
<details><summary>final.py</summary> is a file needed for the competition</details>
<details><summary>stack.py</summary> is responsible for stacking a block given that the robot is already holding a block. It was critical that this is done precicely (otherwise we may knock over the existing stack) but also rapidly (a lot of time can be lost if we make the stacking precise by slowing down the process).</details>
<details><summary>static_block.py</summary> is responsible for grabbing the blocks on the static platforms. We observed that after grabbing the existing blocks on the rotating platform, we have extra time for pikcing up blocks from the static platforms, and so we could focus on grabbing the blocks on the static table in such a way that we can stack the blocks with the white face facing up (more points).</details>
