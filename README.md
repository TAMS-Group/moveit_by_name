### MoveIt By Name

Simple wrapper node that accepts commands on a topic and moves joint model groups to named states.

The main use-case of this wrapper is to reduce delays loading the robot, when you really just want to move an arm to a known pose before/after running your own code.
