# Motion planning in OPENRave environment

![](A_star.gif)



- Implemented A* that ensures collision free trajectory and shortest path from start to goal configuration using 8-connected space and Euclidean distance heuristics for 3 DOF active PR2 robot in OpenRAVE environment.
- Implemented optimal ANA* algorithm that improves heuristic cost function and compared with A* with respect to computation time and net cost function improvement. 
- Implemented RRT-Connect sampling-based algorithm in C++ using nearest-neighbor function for 7 DOF configuration space of the robot’s arm to find a path to specified goal configuration. 
- Goal biasing is selected to yield lowest computation time and shortcut smoothing algorithm is implemented to shorten the path. 
- Implemented Bi-Directional RRT-connect algorithm with shortcut smoothening and compared with RRT-Connect
- C++ plugin is developed in OpenRAVE to send commands from python script. 


