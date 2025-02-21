Position Test
=============

Prints position of vehicle.

## Build
```bash
cd ~/control_vehicles && catkin_make && catkin_make install
cd ~/control_vehicles/src/position_test
mkdir build
cd build
cmake ..
make
```

## Run
1. Start GulliView and `gv_client`.
2. `rosrun position_test position_printer_node.py`
