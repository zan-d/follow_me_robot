This repository provides a simple teleoperation node for the RobAIR robotics courses. Unlike the old ROS1 teleoperation node, this node generates velocity commands by following a user-specified acceleration slope.
This is useful to generate smooth acceleration and deceleration motion on the new ROS2 RobAIR, where the low-level acceleration limit has been removed from the motor controller board (MD49).
Max velocities can be changed at runtime.
Accceleration / deceleration slope value can be changed in the node's source code.
