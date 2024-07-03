Dependencies : Eigen3

How to use
1. Match IP with the canine in include/Setup.hpp
2. Write your codes in the highController function in main.cpp
- List of commands to change the FSM of the controller is in include/CommandLists.hpp
- Check communication_description.txt to get the lists of the robot's states.
  - UDP command lists (from middleware to controller)
  - TCP command lists (from canine to middle ware)

