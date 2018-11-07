# Rapidly exploring Random Belief Tree (RRBT)

**Description:**
This code implements a rapidly exploring random belief tree (RRBT) algorithm, with slight modifications to the implementation first presented in [1].

RRBT is a path planning algorithm that attempts to perform belief space planning, or uncertainty aware planning [1]. The RRBT algorithm is responsible for generating a path that reduces localization uncertainty. For simplicity, this code uses a beacon model for estimating localization uncertainty: the closer the robot gets to the beacon, the better its accuracy. Thus, the RRBT rewires the tree such that the robot gets as close to the beacon as possible to before traveling to its goal.

Code is WIP, with quality improvements and a better README coming soon.

[1] Adam Bry and Nicholas Roy, "Rapidly-exploring random belief trees for motion planning under uncertainty." 2011 IEEE International Conference on Robotics and Automation (ICRA), IEEE, 2011.

**Dependencies:**

* Eigen
* Boost

**Other libraries:**

This code borrows a KD-Tree implementation from https://github.com/jtsiomb/kdtree.  

**kdtree is licensed under the BSD License.**  
Author: John Tsiombikas <nuclear@member.fsf.org>  
kdtree is free software. You may use, modify, and redistribute it under the terms of the 3-clause BSD license.
