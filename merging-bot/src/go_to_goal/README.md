go-to-goal
=============

## Build
```bash
cd ~/control_vehicles && catkin_make && catkin_make install
cd ~/control_vehicles/src/go_to_goal
mkdir build
cd build
cmake ..
make
```

## Run
1. Start GulliView and `gv_client`.
2. `rosrun go_to_goal go_to_goal.py`
